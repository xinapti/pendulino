/*
 * CircularBuffer.h
 *
 *  Created on: 07 nov 2019
 *      Author: alfyhack
 */

#ifndef PROJECT_LIB_CIRCULARBUFFER_CIRCULARBUFFER_H_
#define PROJECT_LIB_CIRCULARBUFFER_CIRCULARBUFFER_H_

#include "opDef.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define real_nElem (nElem + 1) // +1 are the ONE SLOT OPEN

/* In this implementation of the Circular Buffer, to perform the operation correctly, we use the
 * "ONE SLOT OPEN" TECHNICS inside the class, outside the buffer are watch like full buffer.
 * "ONE SLOT OPEN" TECHNICS use the element before the tail as end, and so it are always unreached,
 * but data consistency are safe.
 *
 * ATTENTION: This implementation not override data if the buffer are FULL with normal put, use putF instead
 */
template <class T, uint16_t nElem> class CircularBuffer {
private:
  T buf_[real_nElem];
  uint16_t head_; // Contain next "free" cell
  uint16_t tail_; // Contain last unread cell, exept if (head == tail) in this case buffer are emtpy

public:
  CircularBuffer();
  void memClean();
  void reset();

  // Puts metod, it copy the item inside the buffer and return the old head, return -1 if full
  uint16_t put(T item);
  uint16_t put(T *item);
  uint16_t put(T *item, uint16_t bSize);                      // copy only bSize byte of the *item object (to optimize)
  uint16_t putArray(T *item, uint16_t nItem);                 // copy nItem with bSize (to optimize)
  uint16_t putArray(T *item, uint16_t nItem, uint16_t bSize); // copy nItem with bSize (to optimize)

  // Puts metod, it copy the item inside the buffer and return the old head, override if Full
  uint16_t putF(T item);
  uint16_t putF(T *item);
  uint16_t putF(T *item, uint16_t bSize);                      // copy only bSize byte of the *item object (to optimize)
  uint16_t putFArray(T *item, uint16_t nItem);                 // copy nItem with bSize (to optimize)
  uint16_t putFArray(T *item, uint16_t nItem, uint16_t bSize); // copy nItem with bSize (to optimize)

  // Gets metod
  T get();
  T get(uint16_t *indexRet); // save in *indexRet the index of the returned data
  T *getPtr();
  T *getPtr(uint16_t *indexRet); // save in *indexRet the index of the returned data
  // Special read, to take out char buf in specific structure
  // save in *memDestArray  the value starting from localTail --> localTail+len (obviously in circular buffer logic)
  void memcpyCb(T *memDestArray, uint16_t localTail, uint16_t len);

  //  Get Head Structure information and contenent without change notting
  uint16_t getHead() const;
  T *getHeadPtr(); // Return the memory area where are the current head
  T readHead() const;
  T readFromHeadIndex(uint16_t pos);

  // Get Tail Structure information
  uint16_t getTail() const;
  T *getTailPtr(); // Return the memory area where are the current head
  T readTail() const;
  T readFromTailIndex(uint16_t pos);

public:
  // Structure operation
  // Return the real usable slot information (ONE SLOT OPEN is hiden outside)
  bool isEmpty() const; // true if isEmpty
  bool isFull() const;  // true if full (ONE SLOT OPEN logic hiden inside)
  uint16_t capacity() const;

  uint16_t countSlotBetween(uint16_t localTail, uint16_t localHead) const;
  uint16_t usedSpace() const;
  uint16_t usedSpaceLinear() const; // Return number of used slot, with linear end
  uint16_t availableSpace() const;
  uint16_t availableSpaceLinear() const; // Return number of un-used slot, with linear end and OFS logic

  // Operation to progress the head,
  // On SUCCESS: return the head before increase,
  // On Fail: return "errorRet" if with the increase go over the limit, and nothing are do
  uint16_t headInc();
  uint16_t headAdd(uint16_t len);
  uint16_t headSet(uint16_t pos); // in case of error can go back

  // Operation to progress the tail, return tail after increase, "errorRet" if tail jump over the head
  uint16_t tailInc();
  uint16_t tailAdd(uint16_t len);
  uint16_t tailSet(uint16_t pos); // in case of error can go back
};

template <class T, uint16_t nElem> CircularBuffer<T, nElem>::CircularBuffer() {
  this->head_ = 0;
  this->tail_ = 0;
  memClean();
}

template <class T, uint16_t nElem> void CircularBuffer<T, nElem>::memClean() {
  for (uint16_t i = 0; i < real_nElem; i++)
    memset(&this->buf_[i], 0, sizeof(T));
  reset();
}

template <class T, uint16_t nElem> inline void CircularBuffer<T, nElem>::reset() { head_ = tail_; }
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Puts metod
template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::put(T item) {
  if (isFull())
    return -1;
  buf_[head_] = item;
  return headInc(); // old head
}

template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::put(T *item) {
  return put(item, sizeof(T)); // old head of -1 if full
}

/* copy only bSize byte of the *item object (to optimize),
 * On success are return old head
 * Fail with -1 if the buffer are full
 * Fail with -2 if bSize is over the sizeOf(T)
 */
template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::put(T *item, uint16_t bSize) {
  if (isFull())
    return -1;
  if (bSize > sizeof(T))
    return -2;
  memcpy((void *)&buf_[head_], item, bSize);
  return headInc(); // old head
}

template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::putArray(T *item, uint16_t nItem) {

  return putArray(item, nItem, sizeof(T));
  if (availableSpace() < nItem)
    return -1;
  int ret = head_; // old head
  int linLen = availableSpaceLinear();
  if (nItem <= linLen) {
    memcpy((void *)&buf_[head_], &item[0], nItem * sizeof(T));
    headAdd(nItem);
  } else {
    memcpy((void *)&buf_[head_], &item[0], linLen * sizeof(T));
    headAdd(linLen);
    memcpy((void *)&buf_[head_], &item[linLen + 1], (nItem - linLen) * sizeof(T));
    headAdd(nItem - linLen);
  }

  return ret;
}
/* copy nItem with bSize (to optimize),
 * On success are return old head
 * Fail with -1 if the buffer will be full with this increase
 * Fail with -2 if bSize is over the sizeOf(T)
 */
template <class T, uint16_t nElem>
uint16_t CircularBuffer<T, nElem>::putArray(T *item, uint16_t nItem, uint16_t bSize) {

  if (availableSpace() < nItem)
    return -1;
  if (bSize > sizeof(T))
    return -2;
  int ret = head_; // old head
  for (int i = 0; i < nItem; i++) {
    memcpy((void *)&buf_[head_], &item[i], bSize);
    headInc();
  }
  return ret; // old head
}

/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Puts Force metod
template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::putF(T item) {
  if (isFull())
    this->get();
  buf_[head_] = item;
  return headInc(); // old head
}

template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::putF(T *item) {
  return putf(item, sizeof(T)); // old head of -1 if full
}

/* copy only bSize byte of the *item object (to optimize),
 * On success are return old head
 * Fail with -1 if the buffer are full
 * Fail with -2 if bSize is over the sizeOf(T)
 */
template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::putF(T *item, uint16_t bSize) {
  if (bSize > sizeof(T))
    return -2;
  if (isFull())
    this->get();
  memcpy((void *)&buf_[head_], item, bSize);
  return headInc(); // old head
}

template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::putFArray(T *item, uint16_t nItem) {
  return putFArray(item, nItem, sizeof(T));
}
/* copy nItem with bSize (to optimize),
 * On success are return old head
 * Fail with -1 if the buffer will be full with this increase
 * Fail with -2 if bSize is over the sizeOf(T)
 */
template <class T, uint16_t nElem>
uint16_t CircularBuffer<T, nElem>::putFArray(T *item, uint16_t nItem, uint16_t bSize) {
  if (bSize > sizeof(T))
    return -2;
  int ret = head_; // old head
  for (int i = 0; i < nItem; i++) {
    if (isFull())
      this->get();
    memcpy((void *)&buf_[head_], &item[i], bSize);
    headInc();
  }
  return ret; // old head
}

/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Gets method

template <class T, uint16_t nElem> T CircularBuffer<T, nElem>::get() {
  if (isEmpty()) {
    return T();
  }
  // Read data and advance the tail (we now have a free space)
  auto val = readTail();
  this->tailInc();
  return val;
}

template <class T, uint16_t nElem> T CircularBuffer<T, nElem>::get(uint16_t *indexRet) {
  *indexRet = getTail();
  return get();
}

template <class T, uint16_t nElem> T *CircularBuffer<T, nElem>::getPtr() {
  if (isEmpty()) {
    return nullptr;
  }
  // Read data and advance the tail (we now have a free space)
  auto val = getTailPtr();
  this->tailInc();
  return val;
}

template <class T, uint16_t nElem> T *CircularBuffer<T, nElem>::getPtr(uint16_t *indexRet) {
  *indexRet = getTail();
  return getPtr();
}

template <class T, uint16_t nElem>
void CircularBuffer<T, nElem>::memcpyCb(T *memDestArray, uint16_t localTail, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    memDestArray[i] = buf_[(localTail + i) % real_nElem];
  }
}
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Get Head Structure information

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::getHead() const { return head_; }
template <class T, uint16_t nElem> inline T *CircularBuffer<T, nElem>::getHeadPtr() { return &buf_[head_]; }
template <class T, uint16_t nElem> inline T CircularBuffer<T, nElem>::readHead() const { return buf_[head_]; }
template <class T, uint16_t nElem> inline T CircularBuffer<T, nElem>::readFromHeadIndex(uint16_t pos) {
  uint16_t elemPos = tail_;
  if(pos < capacity())
   elemPos = modSub(head_-1, pos, nElem);
  //TODO: Maybe check if over the head
  return buf_[elemPos];
}

// Get Tail Structure information
template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::getTail() const { return tail_; }
template <class T, uint16_t nElem> inline T *CircularBuffer<T, nElem>::getTailPtr() { return &buf_[tail_]; }
template <class T, uint16_t nElem> inline T CircularBuffer<T, nElem>::readTail() const { return this->buf_[tail_]; }
template <class T, uint16_t nElem> inline T CircularBuffer<T, nElem>::readFromTailIndex(uint16_t pos) {
  uint16_t elemPos = head_-1;
  if(pos < capacity())
    elemPos = modAdd(tail_, pos, nElem);
  //TODO: Maybe check if over the head
  return buf_[elemPos];
}


/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Structure operation

template <class T, uint16_t nElem> inline bool CircularBuffer<T, nElem>::isEmpty() const { return head_ == tail_; }

template <class T, uint16_t nElem> inline bool CircularBuffer<T, nElem>::isFull() const {
  return head_ == modSub(tail_, 1, real_nElem);
}

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::capacity() const { return nElem; }

/* This function return how many block are inside the space pointed by localTail e localHead
 * Local Tail := first unread slot
 * Local Head := first "Free slot" (slot not neaded)
 *       #############  "F"
 * | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
 *       ^               ^
 *   Local Tail       localHead
 *   countSlotBetween = 4
 */
template <class T, uint16_t nElem>
inline uint16_t CircularBuffer<T, nElem>::countSlotBetween(uint16_t localTail, uint16_t localHead) const {
  return modSub(localHead, localTail, real_nElem); // One Free Slot Logic
}

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::usedSpace() const {
  // This 2 block are only to speed-up, formula always function
  if (isFull())
    return nElem;
  if (isEmpty())
    return 0;

  return countSlotBetween(tail_, head_);
}

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::usedSpaceLinear() const {
  if (tail_ <= head_)
    return head_ - tail_; // include empty case

  return real_nElem - tail_; // the end of the array are reach before the end of buffered
}

template <class T, uint16_t nElem> uint16_t CircularBuffer<T, nElem>::availableSpace() const {
  return nElem - usedSpace();
}

// return space between Head and last VALID array index position
// VALID is respect tail and respect one free slot logic
template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::availableSpaceLinear() const {
  if (head_ < tail_) // the end of the buffer are reach before the end of array
    return availableSpace();
  if (tail_ == 0)         // One Slot Free are at the-end of the array
    return nElem - head_; // is like the last slot are ouside the buffer

  return real_nElem - head_;
}

/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Operation to progress the head, return the head before increase
template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::headInc() {
  return headAdd(1); // old head
}

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::headAdd(uint16_t len) {
  if (availableSpace() < len)
    return -1;
  uint16_t oldHead = head_;
  head_ = (head_ + len) % real_nElem;
  return oldHead; // old head
}

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::headSet(uint16_t pos) {
  uint16_t posOld = head_;
  head_ = pos;
  return posOld;
}

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::tailInc() { return tailAdd(1); }

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::tailAdd(uint16_t len) {
  if (usedSpace() < len)
    return -1;
  tail_ = (tail_ + len) % real_nElem;
  return tail_;
}

template <class T, uint16_t nElem> inline uint16_t CircularBuffer<T, nElem>::tailSet(uint16_t pos) {
  uint16_t posOld = tail_;
  tail_ = pos;
  return posOld;
}

#endif /* PROJECT_LIB_CIRCULARBUFFER_CIRCULARBUFFER_H_ */
