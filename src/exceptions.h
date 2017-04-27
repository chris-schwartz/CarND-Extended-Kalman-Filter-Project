//
//  exceptions.h
//  ExtendedKF
//
//  Created by Chris Schwartz on 4/14/17.
//
//

#ifndef exceptions_h
#define exceptions_h

class illegal_argument_exception : public std::exception
{
    virtual const char* what() const throw()
    {
        return "Illegal or unexpected argument detected.";
    }
};

class illegal_state_exception : public std::exception
{
public:
    
    illegal_state_exception(std::string description) {
        description_ = description;
    }
    
    virtual const char* what() const throw()
    {
        return ("Illegal State: " + description_).c_str();
    }
    
private:
    std::string description_;
};


class division_by_zero_exception : public std::exception
{
    virtual const char* what() const throw()
    {
        return "Cannot complete operation because it requires division by zero";
    }
};

#endif /* exceptions_h */
