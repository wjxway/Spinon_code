// Library for circular buffer
// I intended to use boost::circular_buffer, but I cannot get it to work with the compiler...
// This buffer should only be used as a queue, not stack.

// This simple library is thread safe if
// 1. the buffer size always larger than the content size (overriding in push will never occur)
// 2. there's only one writer (push) and one reader (pop).

#ifndef _CIRCBUFFER_HPP_
#define _CIRCBUFFER_HPP_
#pragma once

#include "Arduino.h"

template <class T>
class Circbuffer
{
public:
    /**
     * @brief Construct a new Circbuffer object
     * @param msize the maximum size of the buffer
     */
    Circbuffer(const uint32_t msize);
    /**
     * @brief Destroy the Circbuffer object
     */
    ~Circbuffer();
    /**
     * @brief push T into the head of the queue
     *
     * @param cont content to push in
     */
    void push(const T cont);
    /**
     * @brief pop an item out at the tail and delete it
     *
     * @return T content
     */
    T pop();
    /**
     * @brief peek the item at the tail without deleting it
     *
     * @return T content
     */
    T peek();

    /**
     * @brief maximum size of the buffer
     */
    uint32_t max_size;
    /**
     * @brief current number of elements
     *
     * @note make it volatile cause it could be changed by interrupts... though not sure whether this is necessary.
     */
    volatile uint32_t n_elem;

private:
    /**
     * @brief start of buffer
     */
    T *start;
    /**
     * @brief end of buffer
     */
    T *end;

    /**
     * @brief current head
     */
    T *head;
    /**
     * @brief current tail
     */
    T *tail;
};

template <class T>
Circbuffer<T>::Circbuffer(const uint32_t msize)
{
    max_size = msize;
    n_elem = 0;

    start = new T[msize];
    end = start + msize - 1;
    head = start;
    tail = start;
}

template <class T>
Circbuffer<T>::~Circbuffer()
{
    delete[] start;
}

template <class T>
void Circbuffer<T>::push(const T cont)
{
    // push element
    *head = cont;

    // increase head
    if (head == end)
        head = start;
    else
        head++;

    // check if overlapped
    if (n_elem == max_size - 1)
    {
        if (tail == end)
            tail = start;
        else
            tail++;
    }
    else
        n_elem++;
}

template <class T>
T Circbuffer<T>::pop()
{
    T temp = *tail;

    // if something inside, pop!
    if (n_elem)
    {
        if (tail == end)
            tail = start;
        else
            tail++;

        n_elem--;
    }

    return temp;
}

template <class T>
T Circbuffer<T>::peek()
{
    return *tail;
}

#endif