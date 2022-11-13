// Library for circular buffer
// I intended to use boost::circular_buffer, but I cannot get it to work with the compiler...
// This buffer should only be used as a queue, not stack.

// This simple library is thread safe if
// 1. the buffer size always larger than the content size (overriding in push will never occur)
// 2. there's only one writer (push) and one reader (pop).

// when the previous condition cannot be satisfied, especially the first one, just try to make sure that you don't peek too old messages.

#ifndef _CIRCBUFFER_HPP_
#define _CIRCBUFFER_HPP_
#pragma once

/**
 * @brief circular buffer
 *        to deal with concurrency issues without the need of lock,
 *        please just don't access near the element too deep inside the queue
 *        or just don't do anything close to peek(max_size-1)
 *
 * @tparam T
 * @tparam msize
 */
template <class T, uint32_t msize>
class Circbuffer
{
public:
    /**
     * @brief Construct a new Circbuffer object
     * @param msize the maximum size of the buffer
     */
    Circbuffer();
    /**
     * @brief Destroy the Circbuffer object
     */
    ~Circbuffer();
    /**
     * @brief push T into the head of the queue
     *
     * @param cont content to push in
     */
    void push(const T &cont);
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
     * @brief peek the index th element.
     *
     * @param index
     * @return T content
     */
    T peek(uint32_t index);

    /**
     * @brief clear all contents inside the buffer
     * 
     * @note it won't actually clear everything, it only reset pointers!
     */
    void clear();

    /**
     * @brief maximum size of the buffer
     */
    const uint32_t max_size;
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
    T start[msize];
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

template <class T, uint32_t msize>
Circbuffer<T, msize>::Circbuffer() : max_size{msize}
{
    n_elem = 0;

    end = start + msize - 1;
    head = start;
    tail = start;
}

template <class T, uint32_t msize>
Circbuffer<T, msize>::~Circbuffer()
{
    delete[] start;
}

template <class T, uint32_t msize>
void Circbuffer<T, msize>::push(const T &cont)
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

template <class T, uint32_t msize>
T Circbuffer<T, msize>::pop()
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

template <class T, uint32_t msize>
T Circbuffer<T, msize>::peek()
{
    return *tail;
}

template <class T, uint32_t msize>
T Circbuffer<T, msize>::peek(uint32_t index)
{
    if (tail + index > end)
        return *(tail + index - max_size);
    else
        return *(tail + index);
}

template <class T, uint32_t msize>
void Circbuffer<T, msize>::clear()
{
    n_elem = 0;

    head = start;
    tail = start;
}

#endif