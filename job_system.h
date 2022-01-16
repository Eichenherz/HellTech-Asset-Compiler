#pragma once
//export module job_system;

#include "core_types.h"

#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <memory>

// NOTE: taken from https://wickedengine.net/2018/11/24/simple-job-system-using-standard-c/
// TODO: lockfree
template <typename T, u64 capacity>
struct ring_buffer
{
    T data[ capacity ];
    u64 head = 0;
    u64 tail = 0;
    std::mutex lock; // this just works better than a spinlock here (on windows)

    inline bool push_back( const T& item )
    {
        bool result = false;
        lock.lock();
        u64 next = ( head + 1 ) % capacity;
        if( next != tail )
        {
            data[ head ] = item;
            head = next;
            result = true;
        }
        lock.unlock();
        return result;
    }
    inline bool pop_front( T& item )
    {
        bool result = false;
        lock.lock();
        if( tail != head )
        {
            item = data[ tail ];
            tail = ( tail + 1 ) % capacity;
            result = true;
        }
        lock.unlock();
        return result;
    }
};

struct batch_args
{
    u32 jobIndex;
    u32 groupIndex;
};

struct job_system
{
    ring_buffer<std::function<void()>, 256> jobPool;
    std::condition_variable wakeCondition;    // used with wakeMutex. Worker threads just sleep when there is no job, and the main thread can wake them up
    std::mutex wakeMutex;    // used in conjunction with the wakeCondition above
    std::atomic<u64> finishedLabel;    // track the state of execution across background worker threads
    u64 currentLabel;    // tracks the state of execution of the main thread
    u32 workerCount;


    inline void Init()
    {
        this->workerCount = std::thread::hardware_concurrency();
        this->finishedLabel.store( 0 );
        this->currentLabel = 0;

        // Create all our worker threads while immediately starting them:
        for( u32 threadID = 0; threadID < this->workerCount; ++threadID )
        {
            std::thread worker(
                [&]()
                {
                    std::function<void()> job; // the current job for the thread, it's empty at start.

                    while( true )
                    {
                        if( this->jobPool.pop_front( job ) )
                        {
                            job();
                            this->finishedLabel.fetch_add( 1 ); // update worker label state
                        }
                        else
                        {
                            // no job, put thread to sleep
                            std::unique_lock<std::mutex> lock( this->wakeMutex );
                            this->wakeCondition.wait( lock );
                        }
                    }
                } );

            worker.detach(); // forget about this thread, let it do it's job in the infinite loop that we created above
        }
    }
    inline void Execute( const std::function<void()>& job )
    {
        // The main thread label state is updated:
        this->currentLabel += 1;

        // Try to push a new job until it is pushed successfully:
        while( !this->jobPool.push_back( job ) )
        {
            // This little helper will not let the system to be deadlocked while the main thread is waiting for something
            this->wakeCondition.notify_one(); // wake one worker thread
            std::this_thread::yield(); // allow this thread to be rescheduled
        }

        this->wakeCondition.notify_one(); // wake one thread
    }
    inline void ExecuteBatch( u32 jobCount, u32 batchSize, const std::function<void( batch_args )>& job )
    {
        if( jobCount == 0 || batchSize == 0 ) return;

        // Calculate the amount of job groups to dispatch (overestimate, or "ceil"):
        u32 groupCount = ( jobCount + batchSize - 1 ) / batchSize;

        // The main thread label state is updated:
        this->currentLabel += groupCount;

        for( u32 gi = 0; gi < groupCount; ++gi )
        {
            // For each group, generate one real job:
            auto jobGroup = [jobCount, batchSize, gi, &job]()
            {
                // Calculate the current group's offset into the jobs:
                u32 groupJobOffset = gi * batchSize;
                u32 groupJobEnd = std::min( groupJobOffset + batchSize, jobCount );

                // Inside the group, loop through all job indices and execute job for each index:
                for( u32 i = groupJobOffset; i < groupJobEnd; ++i )
                {
                    batch_args batchArgs = { .jobIndex = i, .groupIndex = gi };
                    job( batchArgs );
                }
            };

            // Try to push a new job until it is pushed successfully:
            while( !this->jobPool.push_back( jobGroup ) )
            {
                // This little helper will not let the system to be deadlocked while the main thread is waiting for something
                this->wakeCondition.notify_one(); // wake one worker thread
                std::this_thread::yield(); // allow this thread to be rescheduled
            }

            this->wakeCondition.notify_one(); // wake one thread
        }
    }
    inline void Wait()
    {
        while( this->finishedLabel.load() < this->currentLabel )
        {
            // This little helper will not let the system to be deadlocked while the main thread is waiting for something
            this->wakeCondition.notify_one(); // wake one worker thread
            std::this_thread::yield(); // allow this thread to be rescheduled
        }
    }
};


extern std::unique_ptr<job_system> pJobSystem;