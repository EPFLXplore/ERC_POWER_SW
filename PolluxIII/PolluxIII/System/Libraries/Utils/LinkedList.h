/*
 * LinkedList.h
 *
 *  Created on: Jun 9, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_UTILS_LINKEDLIST_H_
#define LIBRARIES_UTILS_LINKEDLIST_H_

#include "Utils/ExceptionTracker.h"
#include "Utils/Operators.h"

#include <stdint.h>
#include <cmsis_os.h>

template <typename T> class Iterator : public ExceptionTracker {
public:
	TRACK_EXCEPTIONS();

	Iterator();
	Iterator(T current, Iterator* next);
	virtual ~Iterator() {}

	T& operator*();
	T* operator->();
	operator bool() const;

	// Prefix increment
	Iterator<T>& operator++();

	// Postfix increment
	Iterator<T> operator++(int);

	bool hasNext();

	template <class U> friend bool operator== (const Iterator<U>& a, const Iterator<U>& b);
	template <class U> friend bool operator!= (const Iterator<U>& a, const Iterator<U>& b);

	Iterator<T>* next;

private:
	T current;
	bool valid;
};


template <typename T> class LinkedList : public ExceptionTracker {
public:
	TRACK_EXCEPTIONS();

	LinkedList();
	~LinkedList();

	bool add(T item);
	bool remove(T item);
	bool remove(uint32_t index);
	T pop();
	bool isEmpty();
	uint32_t size();
	Iterator<T> it();

private:
	osMutexId mutex;
	Iterator<T>* iterator;
	uint32_t nitems;

	void removeChild(Iterator<T>* parent);
};


template <class T> Iterator<T>::Iterator() : next(nullptr), current(), valid(false) {

}

template <class T> Iterator<T>::Iterator(T current, Iterator<T>* next) : next(next), current(current), valid(true) {

}

template <class T> T& Iterator<T>::operator*() {
	return this->current;
}

template <class T> T* Iterator<T>::operator->() {
	return &this->current;
}

template <class T> Iterator<T>::operator bool() const {
	return valid;
}

template <class T> bool Iterator<T>::hasNext() {
	return this->next != nullptr;
}

// Prefix increment
template <class T> Iterator<T>& Iterator<T>::operator++() {
	if(hasNext()) {
		this->current = this->next->current;
		this->next = this->next->next;
	} else {
		this->valid = false;
	}

	return *this;
}

// Postfix increment
template <class T> Iterator<T> Iterator<T>::operator++(int) {
	Iterator tmp = *this;

	if(hasNext()) {
		this->current = this->next->current;
		this->next = this->next->next;
	} else {
		this->valid = false;
	}

	return tmp;
}

template <class U> bool operator== (const Iterator<U>& a, const Iterator<U>& b) {
	return a.current == b.current && a.next == b.next;
};

template <class U> bool operator!= (const Iterator<U>& a, const Iterator<U>& b) {
	return a.current != b.current || a.next != b.next;
};

template <class T> LinkedList<T>::LinkedList() {
	const osMutexDef_t mutex_attributes = {};

	this->mutex = osMutexCreate(&mutex_attributes);

	if(mutex == nullptr) {
		throwException("MutexAllocationFailure");
	}
}

template <class T> LinkedList<T>::~LinkedList() {
	osMutexDelete(mutex);
}

#include "Debug/Debug.h"
template <class T> bool LinkedList<T>::add(T item) {
	if(osMutexWait(mutex, 100) == osOK) {
		Iterator<T>* child = this->iterator;

		this->iterator = new Iterator<T>(item, child);

		if(this->iterator != nullptr) {
			this->iterator->trackExceptions(this);
			nitems++;
			osMutexRelease(mutex);
		} else {
			throwException("MemoryAllocationFailure");
			osMutexRelease(mutex);
			return false;
		}

		return true;
	} else {
		throwException("DeadLock");
		return false;
	}
}

template <class T> T LinkedList<T>::pop() {
	if(!isEmpty()) {
		Iterator<T> iter = it();
		T item = *iter;
		remove(0);
		return item;
	} else {
		throwException("EmptyListError");
		return { 0 };
	}
}

template <class T> bool LinkedList<T>::remove(T item) {
	if(osMutexWait(mutex, 100) == osOK) {
		Iterator<T>* parent = nullptr;

		Iterator<T> child = it();

		while(child) {
			if(*child == item) {
				if(parent != nullptr) {
					// This is a non-root node
					removeChild(parent);
				} else {
					// This is the root node
					if(child.hasNext()) {
						Iterator<T>* current_ptr = this->iterator;
						this->iterator = &(++child);
						delete current_ptr;
					} else {
						delete iterator;
						this->iterator = nullptr; // List is now empty
					}
				}

				nitems--;

				osMutexRelease(mutex);
				return true;
			}

			parent = &child;
			child++;
		}

		osMutexRelease(mutex);
		return false;
	} else {
		throwException("DeadLock");
		return false;
	}
}

template <class T> bool LinkedList<T>::remove(uint32_t index) {
	if(osMutexWait(mutex, 100) == osOK) {
		Iterator<T>* parent = nullptr;
		Iterator<T> child = it();

		uint32_t i = 0;

		 while(child) {
			if(i == index) {
				if(parent != nullptr) {
					// This is a non-root node
					removeChild(parent);
				} else {
					// This is the root node
					if(child.hasNext()) {
						Iterator<T>* current_ptr = this->iterator;
						this->iterator = current_ptr->next;
						delete current_ptr;
					} else {
						delete iterator;
						this->iterator = nullptr; // List is now empty
					}
				}

				nitems--;

				osMutexRelease(mutex);
				return true;
			}

			parent = &child;
			child++;
			i++;
		}

		osMutexRelease(mutex);
		return false;
	} else {
		throwException("DeadLock");
		return false;
	}
}

template <class T> void LinkedList<T>::removeChild(Iterator<T>* parent) {
	Iterator<T>* removedChild = parent->next;
	parent->next = removedChild->next;
	delete removedChild;
}

template <class T> uint32_t LinkedList<T>::size() {
	return nitems;
}

template <class T> bool LinkedList<T>::isEmpty() {
	return this->iterator == nullptr;
}

template <class T> Iterator<T> LinkedList<T>::it() {
	if(iterator != nullptr) {
		return *this->iterator;
	} else {
		return Iterator<T>();
	}
}

#endif /* LIBRARIES_UTILS_LINKEDLIST_H_ */
