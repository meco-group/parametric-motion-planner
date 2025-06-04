#ifndef __EXCEPTIONS__
#define __EXCEPTIONS__

#include <exception>
#include <string>

class InvalidEnvironmentOperationException : public std::exception {
    public:
        InvalidEnvironmentOperationException(std::string message) : message_(message) {}

        const char* what() const throw(){
            return message_.c_str();
        }

    private:
        std::string message_;
};

class InvalidPositionInEnvironmentException : public std::exception {
    public:
        InvalidPositionInEnvironmentException(std::string message) : message_(message) {}

        const char* what() const throw(){
            return message_.c_str();
        }

    private:
        std::string message_;
};

class FullCorridorSequenceException : public std::exception {
    public:
        FullCorridorSequenceException() {}

        const char* what() const throw(){
            return message_.c_str();
        }

    private:
        std::string message_ = "Cannot add another corridor to the sequence";
};

class InvalidCorridorSequenceOperationException : public std::exception {
    public:
        InvalidCorridorSequenceOperationException(std::string message) : message_(message) {}

        const char* what() const throw(){
            return message_.c_str();
        }

    private:
        std::string message_;
};

class UnableToPlanEmergencyBrakingTrajectoryException : public std::exception {
    public:
        UnableToPlanEmergencyBrakingTrajectoryException() {}

        const char* what() const throw(){
            return message_.c_str();
        }

    private:
        std::string message_ = "Planner was unable to compute a safe emergency braking trajectory";
};

class UnableToFindWaitingPoint : public std::exception {
    public:
        UnableToFindWaitingPoint(std::string message) : message_(message) {}
        
        const char* what() const throw(){
            return message_.c_str();
        }
    private:
        std::string message_;
};

class AgentCannotWaitWhileAlreadyWaiting : public std::exception {
    public:
        AgentCannotWaitWhileAlreadyWaiting(std::string message) : message_(message) {}

        const char* what() const throw(){
            return message_.c_str();
        }
    private:
        std::string message_;
};

#endif