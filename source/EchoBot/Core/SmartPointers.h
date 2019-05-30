//
// Created by androst on 30.05.19.
//

#ifndef ECHOBOT_SMARTPOINTERS_H
#define ECHOBOT_SMARTPOINTERS_H

#pragma once
#include <memory>

#define ECHOBOT_OBJECT(className)                                  \
    public:                                                     \
        typedef SharedPointer<className> pointer;               \
        static SharedPointer<className> New() {                       \
            SharedPointer<className> smartPtr(new className());   \
            smartPtr->setPtr(smartPtr);                              \
                                                                \
            return smartPtr;                                    \
        }                                                       \
        virtual std::string getNameOfClass() const {            \
            return std::string(#className);                     \
        };                                                      \
        static std::string getStaticNameOfClass() {             \
            return std::string(#className);                     \
        };                                                      \
    private:                                                    \
        void setPtr(className::pointer ptr) {                   \
            mPtr = ptr;                                         \
        }                                                       \



#ifdef WIN32

namespace echobot {

template<class T>
class SharedPointer : public std::shared_ptr<T> {
    using std::shared_ptr<T>::shared_ptr; // inherit constructor

};

}; // end namespace echobot

// Custom hashing functions for the smart pointers so that they can be used in unordered_map etc.
namespace std {
template <class U>
class hash<echobot::SharedPointer<U> >{
    public:
        size_t operator()(const echobot::SharedPointer<U> &object) const {
            return (std::size_t)object.get();
        }
};

} // end namespace std

#else

namespace echobot {

    template<class T>
    using SharedPointer = std::shared_ptr<T>;

}

#endif


#endif //ECHOBOT_SMARTPOINTERS_H
