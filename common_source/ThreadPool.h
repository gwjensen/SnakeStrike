#ifndef ThreadPool_h
#define ThreadPool_h

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <queue>
#include <boost/thread.hpp>

typedef std::unique_ptr<boost::asio::io_service::work> asio_worker;

struct ThreadPool
{
    ThreadPool( uint32_t threads )
        : mService(),
          mWorking( new asio_worker::element_type( mService ) )
    {
        while( threads-- ){
            auto worker = boost::bind( &boost::asio::io_service::run, &(this->mService) );
            mG.add_thread( new boost::thread( worker ));
        }
    }

    template< class C >
        void enqueue( C c ){
            mService.post( c );
        }

    ~ThreadPool() {
            mWorking.reset();
            mG.join_all();
            mService.stop();
    }

    private:
    boost::asio::io_service mService;
    asio_worker mWorking;
    boost::thread_group mG; //Need to keep track of threads for the join used later

};

class ThreadPoolNoASIO
{
    private:
        std::queue< boost::function< void() > > mTasks;
        boost::thread_group mThreads;
        std::size_t mAvailable;
        boost::mutex mMutex;
        boost::condition_variable mCondition;
        bool mIsRunning;

    public:
        /// @brief Constructor.
        ThreadPoolNoASIO( std::size_t pool_size )
        : mAvailable( pool_size ),
          mIsRunning( true )
        {
            for (std::size_t i = 0; i < pool_size; ++i)
            {
                mThreads.create_thread( boost::bind( &ThreadPoolNoASIO::pool_main, this ) ) ;
            }
        }

        /// @brief Destructor.
        ~ThreadPoolNoASIO()
        {
            // Set running flag to false then notify all threads.
            {
                boost::unique_lock< boost::mutex > lock( mMutex );
                mIsRunning = false;
                mCondition.notify_all();
            }

            try
            {
                mThreads.join_all();
            }
            // Suppress all exceptions.
            catch (const std::exception&)
            {

            }
        }

        /// @brief Add task to the thread pool if a thread is currently available.
        template < typename Task >
        void run_task( Task task )
        {
            boost::unique_lock< boost::mutex > lock( mMutex );

            // If no threads are available, then return.
            if ( 0 == mAvailable )
            {
                return;
            }

            // Decrement count, indicating thread is no longer available.
            --mAvailable;

            // Set task and signal condition variable so that a worker thread will
            // wake up andl use the task.
            mTasks.push( boost::function< void() >( task ) );
            mCondition.notify_one();
        }

    private:
        /// @brief Entry point for pool threads.
        void pool_main()
        {
            while (mIsRunning)
            {
                // Wait on condition variable while the task is empty and the pool is
                // still running.
                boost::unique_lock< boost::mutex > lock( mMutex );
                while (mTasks.empty() && mIsRunning)
                {
                    mCondition.wait( lock );
                }
                // If pool is no longer running, break out.
                if (!mIsRunning)
                {
                    break;
                }

                // Copy task locally and remove from the queue.  This is done within
                // its own scope so that the task object is destructed immediately
                // after running the task.  This is useful in the event that the
                // function contains shared_ptr arguments bound via bind.
                {
                    boost::function< void() > task = mTasks.front();
                    mTasks.pop();

                    lock.unlock();

                    // Run the task.
                    try
                    {
                        task();
                    }
                    // Suppress all exceptions.
                    catch (const std::exception&)
                    {

                    }
                }
                // Task has finished, so increment count of available threads.
                lock.lock();
                ++mAvailable;
            } // while running_
        }
};

#endif // ThreadPool_h
