#pragma once
#include <vector>
#include <list>

//template <class T>
class IndexedList {	
	std::vector<int&> index;
	std::list<int> list;

	void push_back(int thing) {
		list.push_back(thing);
		index.push_back(list.back());
	}

	int& operator[](size_t i) {
		return index[i];
	}

};