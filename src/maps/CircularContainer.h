#pragma once

#include <deque>
#include <iostream>

template <class dataType, int maxSize>
class CircularContainer
{
  public:
    CircularContainer()
    {
        if (maxSize <= 0) {
            std::cout << __PRETTY_FUNCTION__ << "Invalid container size.\n";
        }
    }

    void add(const dataType& t)
    {
        if (_data.size() == maxSize) {
            _data.pop_front();
        }
        _data.push_back(t);
    }

    const std::deque<dataType>& getDeque() const { return _data; }

    const std::vector<dataType> getVector() const
    {
        std::vector<dataType> vec;
        vec.reserve(_data.size());
        std::copy(_data.begin(), _data.end(), std::back_inserter(vec));
        return vec;
    }

    void clear()
    {
        if (!_data.empty()) {
            _data.clear();
        }
    }

  private:
    std::deque<dataType> _data;
};
