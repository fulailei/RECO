#ifndef WIR_APPEXCEPTION_H
#define WIR_APPEXCEPTION_H

#include <exception>
#include <string>

class PLCCommException : public std::exception{
private:
    std::string str;
public:
    PLCCommException(std::string s){
        str=s;
    }
    const char * what () const throw () {
         return str.c_str();
      }
};

class PLCException : public std::exception{
private:
    std::string str;
public:
    PLCException(std::string s){
        str=s;
    }
    const char * what () const throw () {
         return str.c_str();
      }
};

class RbComException : virtual public std::exception{
private:
    std::string str;
public:
    RbComException(std::string s){
        str = s;
    }
    const char * what() const throw(){
        return str.c_str();
    }
};

class RbException : virtual public std::exception{
private:
    std::string str;
public:
    RbException(std::string s){
        str = s;
    }
    const char * what() const throw(){
        return str.c_str();
    }
};

#endif
