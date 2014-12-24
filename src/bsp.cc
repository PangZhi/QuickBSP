// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#include "quickbsp/bsp.h"

#include <sched.h>
#include <iostream>

namespace bsp {
    
// global init data
InitData *init_data_ptr = NULL;
// message type for bsp
const tmb::message_type_id message_type = 0;

void BspSpmd(ThreadData* thread_data_ptr) {
  // bind core and thread
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(thread_data_ptr->pid, &mask);
  if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0) {
      std::cout << "warning: could not set CPU affinity in cpu "<< thread_data_ptr->pid << "continuing..." << std::endl;
  }
  // run the spmd
  init_data_ptr->spmd()(thread_data_ptr, init_data_ptr->this_ptr());
}

void BspBegin(spmd_fun spmd, void *this_ptr, const int thread_num) {
  init_data_ptr = new InitData(spmd, this_ptr, thread_num);
  // bind core and thread
  cpu_set_t mask;
  CPU_ZERO(&mask);
  // you can set the hyper thread you want to bind
  CPU_SET(0, &mask);
  if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0) {
      std::cout << "warning: could not set CPU affinity, continuing..." << std::endl;
  }

  ThreadData *this_thread_data_ptr = NULL;
  for (int i = 0; i < thread_num; i++) {
      // construct thread data structure for every thread
      tmb::client_id msg_bus_id = init_data_ptr->msg_bus_ptr()->Connect();
      ThreadData *thread_data_ptr = new ThreadData(i, msg_bus_id, thread_num);
      init_data_ptr->SetMessageBusId(i, msg_bus_id);
      tmb::message_type_id msg_t_id = 0;
      if ( !init_data_ptr->msg_bus_ptr()->RegisterClientAsSender(msg_bus_id, msg_t_id)
          || !init_data_ptr->msg_bus_ptr()->RegisterClientAsReceiver(msg_bus_id, msg_t_id)) {
          std::cout << "ERROR: Thread" << i << " " << "fail to register with msg bus" << std::endl;
          exit(-1);
      }
      if (i == 0) {
          this_thread_data_ptr = thread_data_ptr;
      } else {
          // update init data
          init_data_ptr->add_thread(i, std::thread(bsp::BspSpmd, thread_data_ptr));
      }
  }
  init_data_ptr->spmd()(this_thread_data_ptr, init_data_ptr->this_ptr());
}


void BspEnd(ThreadData *thread_data_ptr) {
  if (thread_data_ptr->pid != 0) {
      if (!init_data_ptr->msg_bus_ptr()->
              Disconnect(thread_data_ptr->msg_bus_id)) {
          std::cout << "ERROR: Thread" << thread_data_ptr->pid << " " << "fail to disconnect" << std::endl;
      }
      delete thread_data_ptr;
      return;
  }
  
  // main thread should wait for all threads to join
  std::cout << "thread " << thread_data_ptr->pid << " waiting" << std::endl;
  for (int i = 1; i < init_data_ptr->thread_num(); i++) {
      init_data_ptr->GetThreadByIndex(i).join();
  }
  if (!init_data_ptr->msg_bus_ptr()
          ->Disconnect(thread_data_ptr->msg_bus_id)) {
      std::cout << "ERROR: Thread" << thread_data_ptr->pid << " " << "fail to disconnect" << std::endl;
  }
  delete thread_data_ptr; 
  delete init_data_ptr;
  init_data_ptr = NULL;
}

void BspSync(ThreadData *thread_data_ptr) {

  // synchronization, threads will be blocked until the last thread has finished its word
  init_data_ptr->add_sync_num();

  // swap the read/write buffer
  void **tmp = thread_data_ptr->pub_w;
  thread_data_ptr->pub_w = thread_data_ptr->pub_r;
  thread_data_ptr->pub_r = tmp;

}

void BspSend(ThreadData *thread_data_ptr, const int dest, const void *msg, const std::size_t bytes) {
  tmb::TaggedMessage t_msg(msg, bytes, message_type);
  tmb::Address address;
  address = address.AddRecipient(init_data_ptr->GetMessageBusIdByIndex(dest));
  tmb::MessageStyle msg_st;

  tmb::MessageBus::SendStatus stat = init_data_ptr->msg_bus_ptr()->Send(thread_data_ptr->msg_bus_id,
          address,
          msg_st,
          std::move(t_msg),
          tmb::kMaxAsyncPriority-init_data_ptr->superstep_loc());

  if (stat != tmb::MessageBus::SendStatus::kOK) {
      std::cout << static_cast<int>(stat) << std::endl;
      std::cout << "ERROR: fail to send message" << std::endl;
      exit(-1);
  }
}


void BspSendToAll(ThreadData* thread_data_ptr, const void *msg, const std::size_t bytes) {
  tmb::TaggedMessage t_msg(msg, bytes, message_type);
  tmb::Address address;
  address = address.All(true);
  tmb::MessageStyle msg_st;
  msg_st = msg_st.Broadcast(true);
  tmb::MessageBus::SendStatus stat = init_data_ptr->msg_bus_ptr()->Send(thread_data_ptr->msg_bus_id,
          address,
          msg_st,
          std::move(t_msg),
          tmb::kMaxAsyncPriority-init_data_ptr->superstep_loc());

  if (stat != tmb::MessageBus::SendStatus::kOK) {
      std::cout << static_cast<int>(stat) << std::endl;
      std::cout << "ERROR: fail to send message" << std::endl;
      exit(-1);
  }
}


bool BspMove(ThreadData* thread_data_ptr, tmb::TaggedMessage& msg) {
  tmb::AnnotatedMessage *a_msg_ptr = new tmb::AnnotatedMessage;

  // try to receive from last superstep, if there is any message
  // ovewrite msg
  if (init_data_ptr->msg_bus_ptr()->ReceiveIfAvailable(thread_data_ptr->msg_bus_id,
              a_msg_ptr,
              tmb::kMaxAsyncPriority-init_data_ptr->superstep_loc()+1)) {
      msg = std::move(a_msg_ptr->tagged_message);
      delete a_msg_ptr;
      return true;
  } else {
      // std::cout << "thread " << BspPid() << " doesn't have message" << std::endl;
      delete a_msg_ptr;
      return false;
  }
}

void BspSetHalt(const bool to_halt) {
  init_data_ptr->set_halt(to_halt);
}

bool BspGetHalt() {
  return init_data_ptr->halt();
}

int BspGetSuperstep() {
  return init_data_ptr->superstep_loc();
}
}  // namespace bsp
