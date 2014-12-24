// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#ifndef QUICKBSP_COMMON_H_
#define QUICKBSP_COMMON_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "tmb/message_bus.h"
#include "tmb/pure_memory_message_bus.h"

namespace bsp {

/**
* @brief This file contains the basic structure for thread control in bsp.
*/

/*
* @brief the data of a single thread, every thread has his own copy
**/
struct ThreadData{
  /**
  * @brief basic constructor
  */
  ThreadData(const int pid_, const tmb::client_id msg_bus_id_, const int thread_num_)
      : pid(pid_),
        msg_bus_id(msg_bus_id_) {
    pub_w = new void*[thread_num_];
    pub_r = new void*[thread_num_];
  }

  /**
  * @brief basic distructor
  */
  ~ThreadData() {
    delete[] pub_w;
    pub_w = NULL;
    delete[] pub_r;
    pub_r = NULL;
  }

  // thread id
  int pid;
  // client id in message
  tmb::client_id msg_bus_id;
  // to minimize communication, we just transfer pointer to data.
  // pub_w is local write buffer, its length is thread_num, this thread write content
  // to thread 0...k in pub_w[0]...pub_w[k]
  void **pub_w;
  // local read buffer
  void **pub_r;
};

/**
* @brief define a function type
*/
typedef void (*spmd_fun)(ThreadData*, void*);

/**
* @brief the global data. All the threads share one copy
*/
class InitData{
 public:
  /**
  * @brief constructor
  */
  InitData(const spmd_fun spmd, void *this_ptr, const int thread_num)
      : thread_num_(thread_num),
        sync_num_(0),
        superstep_loc_(0),
        halt_(false),
        vote_halt_(true),
        spmd_(spmd),
        this_ptr_(this_ptr) {
    msg_bus_ptr_ = std::unique_ptr<tmb::PureMemoryMessageBus>(new tmb::PureMemoryMessageBus());
    threads_.resize(thread_num);
    msg_bus_ids_.resize(thread_num);
  }

  /**
  * @brief add synchronization number to indicate that another thread has entered into barrier synchronization.
  *      If it is the last thread to enter into barrier synchronization, it should reset some values and 
  *      wake up other threads; otherwise just wait
  */
  void add_sync_num() {
    std::unique_lock<std::mutex> lock(mutex_);
    ++sync_num_;
    if (sync_num_ == thread_num_) {
      sync_num_ = 0;
      ++superstep_loc_;
      halt_ = vote_halt_;
      vote_halt_ = true;
      cond_.notify_all();
    } else {
      cond_.wait(lock);
    }
    lock.unlock();
  }

  /**
  * @brief add a thread to init data so keep track of it
  * @param i position of this thread
  * @param t thread to be added
  */
  inline void add_thread(const int i, std::thread&& t) {
    threads_[i] = std::move(t);
  }

  /**
  * @brief get thread by index
  * @return the thread
  */
  inline std::thread& GetThreadByIndex(const int index) {
    return threads_[index];
  }

  /**
  * @brief get message bus id by index
  * @return the message bus id
  */
  inline tmb::client_id GetMessageBusIdByIndex(const int index) {
    return msg_bus_ids_[index];
  }

  /**
   * @brief set message bus id by index
   * @param the index want to set 
   * @param the message bus id value
   */
  inline void SetMessageBusId(const int index, const tmb::client_id msg_bus_id) {
    msg_bus_ids_[index] = msg_bus_id;
  }

  /**
  * @brief get current superstep number
  * @return return current superstep number
  */
  inline int superstep_loc() const {
    return superstep_loc_;
  }

  /**
  * @brief get total number of threads in this program
  * @return number of threads
  */
  inline int thread_num() const {
    return thread_num_;
  }


  /**
  * @brief get halt information
  * @return true if the work is done and all the threads should halt; otherwise false
  */
  inline bool halt() const {
    return halt_;
  }

  /**
  * @brief every thread call this function to vote to halt
  */
  inline void set_halt(const bool to_halt) {
    std::unique_lock<std::mutex> lock(mutex_);
    vote_halt_ = to_halt & vote_halt_;
    lock.unlock();
  }

  /**
  * @brief return the spmd function
  * @return the spmd function
  */
  inline spmd_fun spmd() const {
    return spmd_;
  }

  /**
  * @brief return the class instance pointer which contains the spmd function
  * @brief the class instance pointer which contains the spmd function
  */
  inline void* this_ptr() const {
    return this_ptr_;
  }

  /**
  * @brief return the message bus
  * @return the message bus
  */
  inline std::unique_ptr<tmb::PureMemoryMessageBus>& msg_bus_ptr() {
    return msg_bus_ptr_;
  }

 private:
  // total thread num
  int thread_num_;
  // how many threads have been into sync
  int sync_num_;
  // current super step
  int superstep_loc_;
  // race control
  std::mutex mutex_;
  // race control
  std::condition_variable cond_;
  // all the threads
  std::vector<std::thread> threads_;
  // all the message bus ids
  std::vector<tmb::client_id> msg_bus_ids_;
  // whether to halt
  bool halt_;
  // record the vote result of threads in this superstep
  bool vote_halt_;
  // pointer to the function
  spmd_fun spmd_;
  // pointer to the class instance where spmd function is in
  void *this_ptr_;
  // message bus pointer
  std::unique_ptr<tmb::PureMemoryMessageBus> msg_bus_ptr_;
};

}  // namespace bsp

#endif  // QUICKBSP_COMMON_H_
