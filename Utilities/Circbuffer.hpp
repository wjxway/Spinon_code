/**
 * @file Circbuffer.hpp
 * @author Jinxian Wang
 *
 * @brief Circular Buffer Implementation
 */

// This simple library is thread safe if
// 1. the buffer size always larger than the content size (overriding in push
// will never occur) 2. there's only one writer (push) and one reader (pop).

// when the previous condition cannot be satisfied, especially the first one,
// just try to make sure that you don't peek too old messages.

// this file also contain a copycat class to use these two functions, create a
// original class where only one task can add to it. then create multiple
// copycats where you can pop(). each copycat act like a original, but with no
// extra copy cost. and they won't interfere with each other. perfect for
// duplicating streams of commands.
#ifndef _CIRCBUFFER_HPP_
#define _CIRCBUFFER_HPP_

/**
 * @brief copycat class for circbuffer
 *
 * @tparam T member type
 * @tparam max_size length of circbuffer
 *
 * @note this class is a copycat for Circbuffer class.
 *       On the back of one copycat class, there must be a original Circbuffer,
 *       copycat will not store data, but points you to the original.
 *
 *       You can only access data of the original with copycat, but cannot write.
 *       The goal of copycat class is that you can instantiate a copy without cost,
 *       and then use pop() to keep track of where you left off.
 *
 *       Note that when you call pop() in copycat class, you virtually pop out
 *       the data by moving the head pointer, but will not actually pop out the
 *       data in the original. This means you can have multiple copycats
 *       popping independently. Actually, this is the whole point of creating
 *       this class, you can have multiple 'streams' of commands of the same
 *       actual data without interferening with each other, which is really
 *       helpful for multi-threading: you can have a copycat class for the
 *       stream you want in each thread and just use them as a stream (queue),
 *       without the actual cost of copying multiple streams (queues).
 *
 * @note Because we need to deal with multi-threading here, it's not possible
 *       to check for size() before peek() or pop(). and I do not want to add a
 *       flag to indicate this unless absolutely necessary. So instead I will
 *       assume that this function can only be used with msgs, and for msgs,
 *       their timing information can not be 0. What you should do is: check
 *       for timing information after pop() or peek() to determine whether they
 *       are valid. Yeah I know, I know, this is ugly, but it makes the
 *       interface cleaner. This implementation also means that you cannot use
 *       this class for general class T unless you don't care about
 *       multi-threading.
 */
/**
 * @brief circular buffer
 *        to deal with concurrency issues without the need of lock,
 *        please just don't access near the element too deep inside the queue
 *        or just don't do anything close to peek(max_size-1)
 *
 * @param T type
 * @param max_size size of buffer
 * 
 * @note not sure if we want to put msize here. it will instatiate a new class
 *       for every size... seems redundant. but by giving the size inside
 *       template, we can use plain arrays instead of vectors.
 */
template <class T, uint32_t max_size>
class Circbuffer
{
public:
    /**
     * @brief Construct a new Circbuffer object
     * @param msize the maximum size of the buffer
     */
    Circbuffer()
    {
        n_elem = 0;

        end = start + max_size - 1;
        tail = start;
        head = start;
    }

    /**
     * @brief push T into the tail of the queue
     *
     * @param cont content to push in
     */
    void push(const T &cont)
    {
        io_flag++;

        // push element
        *tail = cont;

        // increase tail
        tail = (tail == end) ? start : (tail + 1);

        // check if overlapped
        if (n_elem == max_size - 1)
        {
            if (head == end)
            {
                head = start;
                head_rounds++;
            }
            else
                head++;
        }
        else
            n_elem++;
    }

    /**
     * @brief (not thread safe!) pop an item out at the head and delete it
     *        (actually you shouldn't use it, you should make a copycat and
     *        pop() on copycat class.)
     *
     * @return T content
     */
    T pop()
    {
        io_flag++;

        T temp = *head;

        // if something inside, pop!
        if (n_elem)
        {
            if (head == end)
            {
                head = start;
                head_rounds++;
            }
            else
                head++;

            n_elem--;
        }

        return temp;
    }

    /**
     * @brief (not thread safe!) peek the item at the head without deleting it
     *
     * @return T content
     *
     * @note why would anyone use this?
     * @note return incorrect result when n_elem=0, be sure to check before call.
     */
    T &peek()
    {
        return *head;
    }

    /**
     * @brief (not thread safe!) peek the index th oldest element.
     *
     * @param index The offset from the head.
     * @return T The existing value
     *
     * @warning Undefined behavior for n_elem = 0. Check before calling.
     */
    T &peek(uint32_t index)
    {
        if (head + index > end)
        {
            return *(head + index - max_size);
        }
        else
        {
            return *(head + index);
        }
    }

    /**
     * @brief (not thread safe!) peek the index th element counting from tail.
     *
     * @param index
     * @return T content
     *
     * @warning Undefined behavior for n_elem = 0. Check before calling.
     */
    T &peek_tail(uint32_t index = 0)
    {
        if (tail < start + index + 1)
        {
            return *(tail + max_size - index - 1);
        }
        else
        {
            return *(tail - index - 1);
        }
    }

    /**
     * @brief Remove all entries in the buffer.
     */
    void clear()
    {
        io_flag++;

        n_elem = 0;

        tail = start;
        head = start;

        // increment head_rounds so that when we clear up a buffer, the copycat
        // will always update accordingly.
        head_rounds += 2;
    }

    /**
     * @brief how many rounds has head move.
     *
     * @return uint32_t # of rounds
     */
    constexpr uint32_t Get_head_rounds()
    {
        return head_rounds;
    }

    /**
     * @brief current number of elements
     *
     * @note make it volatile cause it could be changed by interrupts... though
     * not sure whether this is necessary.
     */
    volatile uint32_t n_elem;

    /**
     * @brief how many times has this been edited
     */
    uint32_t io_flag = 0;

private:
    T start[max_size];
    T *end;
    T *tail;
    T *head;
    uint32_t head_rounds = 0;
};

template <class T, uint32_t max_size>
class Circbuffer_copycat
{
public:
    /**
     * @brief Construct a new Circbuffer_copycat object
     *
     * @param orig1 a pointer to what it's gonna copy
     */
    Circbuffer_copycat(Circbuffer<T, max_size> *orig1) :
        orig(orig1), start(orig1->start), end(orig1->end),
        max_size(orig1->max_size)
    {
        uint32_t curr_flag;
        do
        {
            curr_flag = orig->io_flag;
            head = orig->head;
            head_rounds = orig->head_rounds;
        } while (orig->io_flag != curr_flag);
    }

    /**
     * @brief (thread safe!) pop in the copycat class, note that this will only
     * change copycat's head pointer, but will not change the original class.
     *
     * @return T TODO Describe
     *
     * @note return T{} when n_elem=0, so when used with parsed_msgs, just
     * check the timing to determine whether the buffer is empty (if empty,
     * time=0). I know this is not elegant and using std::pair might be better,
     * but hey I want to keep the interface unified.
     */
    T pop()
    {
        T temp{};
        uint32_t curr_flag, empty = 0;

        bool not_first_time = 0;

        do
        {
            // do not run this part for the first run
            if (not_first_time)
            {
                // reset head
                head = orig->head;
                head_rounds = orig->head_rounds;
            }
            else
                not_first_time = 1;

            // it's not empty as long as head!=tail even if concurrency happens
            // here and make it true, the header must be invalid, so it will loop
            // again.
            if (head != orig->tail)
            {
                empty = 0;

                temp = *head;
                if (head == end)
                {
                    head = start;
                    head_rounds++;
                }
                else
                    head++;
            }
            // when empty, return T{}
            else
                empty = 1;
        } while (head_invalid());

        return empty ? T{} : temp;
    }

    /**
     * @brief (thread safe!) peek the oldest.
     *
     * @return T
     *
     * @note return T{} when n_elem=0, so when used with parsed_msgs, just check the timing to determine whether the buffer is empty (if empty, time=0).
     *       I know this is not elegant and using std::pair might be better, but hey I want to keep the interface unified.
     */
    T peek()
    {
        T temp{};
        uint32_t curr_flag, empty = 0;

        bool not_first_time = 0;

        do
        {
            // do not run this part for the first run
            if (not_first_time)
            {
                // reset head
                head = orig->head;
                head_rounds = orig->head_rounds;
            }
            else
                not_first_time = 1;

            // it's not empty as long as head!=tail even if concurrency happens
            // here and make it true, the header must be invalid, so it will
            // loop again.
            if (head != orig->tail)
            {
                empty = 0;
                temp = *head;
            }
            // when empty, return T{}
            else
                empty = 1;
        } while (head_invalid());

        return empty ? T{} : temp;
    }

    /**
     * @brief how many rounds has the copycat's head go
     *
     * @return uint32_t rounds
     *
     * @note no idea why anyone want to use this
     */
    constexpr uint32_t Get_head_rounds()
    {
        return head_rounds;
    }

    /**
     * @brief get io_flag of the original
     *
     * @return uint32_t io_flag
     */
    constexpr uint32_t Get_io_flags()
    {
        return orig->io_flag;
    }

private:
    Circbuffer<T, max_size> *orig;
    T *start;
    T *end;
    uint32_t max_size;

    /**
     * @brief head of copycat
     */
    T *head;
    /**
     * @brief head of copycat rounds
     */
    uint32_t head_rounds = 0;

    /**
     * @brief check if head of copycat is valid
     *
     * @note it's invalid when: (head_rounds==orig->head_rounds&&head<orig->head)||(head_rounds<orig->head_rounds)
     */
    constexpr bool head_invalid()
    {
        return (head_rounds == orig->head_rounds && head < orig->head)
            || (head_rounds < orig->head_rounds);
    }
};

#endif // _CIRCBUFFER_HPP_
