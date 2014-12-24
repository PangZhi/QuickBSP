// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#ifndef QUICKBSP_BSP_H_
#define QUICKBSP_BSP_H_

#include "quickbsp/common.h"
#include "tmb/address.h"
#include "tmb/message_style.h"
#include "tmb/tagged_message.h"

namespace bsp {

/**
* @brief A library which provides thread-level bulk synchronous parallel model.
*         This file contains all the interface required to realize superstep and barrier sychronization.
*/

/**
* @brief run spmd function, used for all threads except the main thread
* @param thread_data_ptr the thread data of this thread
*/
void BspSpmd(ThreadData *thread_data_ptr);

/**
* @brief begin bsp program
* @param spmd the function every thread should run
* @param this_ptr the this pointer of the class where spmd function is in
* @param thread_num the number of threads to start
*/
void BspBegin(spmd_fun spmd, void *this_ptr, const int thread_num);

/**
* @brief end bsp program, should be in pair with BspBegin()
* 		all the threads should wait until the final thread is ended
* @param thread_data_ptr thread data of this thread
*/
void BspEnd(ThreadData* thread_data_ptr);

/**
* @brief barrier synchronization
* @param thread_data_ptr thread data of this thread
*/
void BspSync(ThreadData* thread_data_ptr);

/**
* @brief send msg to another thread 
* @param thread_data_ptr thread data of this thread
* @param dest pid of the destination
* @param msg pointer to the message to be sent 
* @param bytes length of the msg
*/
void BspSend(ThreadData* thread_data_ptr, const int dest, const void *msg, const std::size_t bytes);

/**
* @brief send msg to all other threads
* @param thread_data_ptr thread data of this thread 
* @param msg point to the message to be sent
* @param bytes length of the message
*/
void BspSendToAll(ThreadData* thread_data_ptr, const void *msg, const std::size_t bytes);

/**
* @brief asychronously get message from message bus, if receive no message, means messages from the last super step have all been recevied
* @param thread_data_ptr thread data of this thread
* @param msg an empty TaggedMessage, it will be overwritten if the thread can get message from message bus
* @return true if there is message, otherwise false
*/
bool BspMove(ThreadData* thread_data_ptr, tmb::TaggedMessage& msg);

/**
* @brief vote to halt
* @param halt whether vote to halt
*/
void BspSetHalt(const bool halt);

/**
* @brief get global halt status
* @return true if all the threads vote to halt. o.w. false.
*/
bool BspGetHalt();

/**
* @brief get current superstep id
* @return the current superstep id
*/
int BspGetSuperstep();

}  // namespace bsp
#endif  // QUICKBSP_BSP_H_
