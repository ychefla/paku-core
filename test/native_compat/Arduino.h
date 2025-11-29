/**
 * @file Arduino.h
 * @brief Arduino compatibility layer for native testing
 * 
 * This provides a minimal implementation of Arduino types and functions
 * needed for unit testing on native platforms.
 */
#pragma once

#include <string>
#include <cstring>
#include <cstdio>

/**
 * @brief Arduino-compatible String class for native testing
 */
class String {
private:
    std::string _str;

public:
    String() : _str() {}
    String(const char* s) : _str(s ? s : "") {}
    String(const String& s) : _str(s._str) {}
    String(const std::string& s) : _str(s) {}
    String(int value) : _str(std::to_string(value)) {}
    String(unsigned int value) : _str(std::to_string(value)) {}
    String(long value) : _str(std::to_string(value)) {}
    String(unsigned long value) : _str(std::to_string(value)) {}
    String(float value) {
        char buffer[64];  // Increased buffer size for safety with extreme values
        snprintf(buffer, sizeof(buffer), "%.2f", value);
        _str = buffer;
    }
    String(double value) {
        char buffer[64];  // Increased buffer size for safety with extreme values
        snprintf(buffer, sizeof(buffer), "%.2f", value);
        _str = buffer;
    }

    const char* c_str() const { return _str.c_str(); }
    unsigned int length() const { return static_cast<unsigned int>(_str.length()); }
    bool isEmpty() const { return _str.empty(); }
    
    String& operator=(const String& rhs) {
        _str = rhs._str;
        return *this;
    }
    
    String& operator=(const char* rhs) {
        _str = rhs ? rhs : "";
        return *this;
    }
    
    String operator+(const String& rhs) const {
        return String(std::string(_str + rhs._str));
    }
    
    String operator+(const char* rhs) const {
        return String(std::string(_str + (rhs ? rhs : "")));
    }
    
    String& operator+=(const String& rhs) {
        _str += rhs._str;
        return *this;
    }
    
    String& operator+=(const char* rhs) {
        if (rhs) _str += rhs;
        return *this;
    }
    
    bool operator==(const String& rhs) const {
        return _str == rhs._str;
    }
    
    bool operator==(const char* rhs) const {
        return _str == (rhs ? rhs : "");
    }
    
    bool operator!=(const String& rhs) const {
        return _str != rhs._str;
    }
    
    int indexOf(const String& s) const {
        size_t pos = _str.find(s._str);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }
    
    int indexOf(const char* s) const {
        if (!s) return -1;
        size_t pos = _str.find(s);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }
    
    String substring(unsigned int beginIndex) const {
        if (beginIndex >= _str.length()) return String();
        return String(_str.substr(beginIndex));
    }
    
    String substring(unsigned int beginIndex, unsigned int endIndex) const {
        if (beginIndex >= _str.length()) return String();
        if (endIndex > _str.length()) endIndex = static_cast<unsigned int>(_str.length());
        if (beginIndex >= endIndex) return String();
        return String(_str.substr(beginIndex, endIndex - beginIndex));
    }
};

// Friend function for concatenation with const char*
inline String operator+(const char* lhs, const String& rhs) {
    return String(lhs) + rhs;
}
