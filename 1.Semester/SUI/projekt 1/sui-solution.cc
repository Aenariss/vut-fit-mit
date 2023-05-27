#include "search-strategies.h"
#include "memusage.h"
#include <deque>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <sstream>

// get string representation from SearchState
std::string stateString(const SearchState &toConvert) {
	std::ostringstream tmp;
	tmp << toConvert;
	return std::move(tmp).str();
}

// find state in unordered set
void insertStringUnordered(std::unordered_set<std::string> *closed, const SearchState &insert) {
	auto tmp = stateString(insert);
	closed->insert(tmp);
}

// return true if found, false otherwise
bool inUnorderedSet(const std::unordered_set<std::string> &searchedSet, const SearchState &searched) {
	auto tmp = stateString(searched);
	return (searchedSet.find(tmp) != searchedSet.end());
}

std::vector<SearchAction> getPath(const SearchState &init, std::unordered_map<std::string, std::pair<std::string, std::vector<SearchAction>>> parents, const std::pair<std::string,std::pair<std::string,SearchAction>> &final) {
	
	std::vector<SearchAction> path;
	std::string current = final.second.first;
	path.push_back(final.second.second);

	const std::string initString = stateString(init);

	// if the last one matches the initial, we didnt move at all
	// otherwise go until we reach the beginning
	while ((initString != current)) {
		std::pair<std::string, std::vector<SearchAction>> prev = parents[current];
		if((final.second.first != prev.first)) {
			path.push_back(prev.second.front());
		}
		current = prev.first;
	}

	// reverse path vector to go from the beginning
	reverse(path.begin(), path.end());
	return path;
}

bool tooMuchMemory(size_t mem_limit_) {
	const int memory_safe = mem_limit_ * 0.12; // ~12% of given ram should be safe enough of a buffer
	return ((getCurrentRSS() + memory_safe) > mem_limit_);	// out of memory
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::unordered_map<std::string, std::pair<std::string, std::vector<SearchAction>>> parents;
	// queue OPEN
	std::deque<SearchState> open;
	open.push_front(init_state);
	// closed
	std::unordered_set<std::string> closed;
	// open set
	std::unordered_set<std::string> open_set;

	// while OPEN is not empty
	while (open.size() != 0) {
		SearchState first_state(open.front());	// current head of the queue
		auto actions = first_state.actions();	// actions possible from the first state in queue
		open.pop_front();	// remove head of the queue
		open_set.erase(stateString(first_state));
		insertStringUnordered(&closed, first_state);	// insert into closed
		
		if (tooMuchMemory(mem_limit_)) {
			break;
		}

		//add all nodes to Open queue
		for (size_t i = 0; i < actions.size(); i++){

			if (tooMuchMemory(mem_limit_)) {
				return {};
			}

			auto action = actions[i];
			auto new_state(action.execute(first_state));	// execute each possible action from "left to right"

			if (!inUnorderedSet(closed, new_state) && !inUnorderedSet(open_set, new_state)) {

				std::pair<std::string,std::pair<std::string,SearchAction>> tmp = std::make_pair(stateString(new_state),std::make_pair(stateString(first_state),action));
				std::vector<SearchAction> actionVector {action};
				parents[stateString(new_state)] = std::make_pair(stateString(first_state), actionVector);

				if (new_state.isFinal()){
					return getPath(init_state, parents, tmp);
				}
				open.push_back(new_state);
				insertStringUnordered(&open_set, new_state);
			}
		}
	}
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	std::unordered_map<std::string, std::pair<std::string, std::vector<SearchAction>>> parents;
	std::unordered_map<std::string, int> depths;	// DLS depth tracking
	depths[stateString(init_state)] = 0;

	// queue OPEN
	std::deque<SearchState> open;
	open.push_front(init_state);
	// closed
	std::unordered_set<std::string> closed;
	// open set
	std::unordered_set<std::string> open_set;

	// while OPEN is not empty
	while (open.size() != 0) {
		SearchState first_state(open.front());	// current head of the queue
		int depth = depths[stateString(first_state)];
		auto actions = first_state.actions();	// actions possible from the first state in queue
		insertStringUnordered(&closed, first_state);	// insert into closed
		open.pop_front();	// remove head of the queue
		open_set.erase(stateString(first_state));

		if (tooMuchMemory(mem_limit_)) {
			break;
		}

		//add all nodes which are not in visited set to Open queue
		for (size_t i = 0; i < actions.size(); i++){
			auto action = actions[i];
			auto new_state(action.execute(first_state));	// execute each possible action from "left to right"

			if (!inUnorderedSet(closed, new_state) && !inUnorderedSet(open_set, new_state) && (depth+1 < depth_limit_)) {

				std::pair<std::string,std::pair<std::string,SearchAction>> tmp = std::make_pair(stateString(new_state),std::make_pair(stateString(first_state),action));
				std::vector<SearchAction> actionVector {action};
				parents[stateString(new_state)] = std::make_pair(stateString(first_state), actionVector);
				depths[stateString(new_state)] = depth+1;

				if (new_state.isFinal()){
					return getPath(init_state, parents, tmp);
				}
				open.push_front(new_state);
				insertStringUnordered(&open_set, new_state);
			}
		}
	}
	return {};
}

int getColor(const Color x) {
	if (x == Color::Diamond || x == Color::Heart) {
		return 0;	// red
	}
	return 1;		// black
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
	
    int cards_out_of_home = 52;
    for (const auto &home : state.homes) {
        auto opt_top = home.topCard();
        if (opt_top.has_value())
            cards_out_of_home -= opt_top->value;
    }

	auto stacks = state.stacks;
	int out_of_order = 0;
	for (size_t i = 0; i < stacks.size(); i++) {	// go through all 8 stacks
		auto storage = stacks[i].storage();
		auto stack_size = storage.size();
		auto max_card = storage[0].value;			// default max is value of the first in stack
		auto prev_color = getColor(storage[0].color);

		for (size_t k = 1; k < stack_size; k++) {	// go through cards in each stack
			auto card_val = storage[k].value;			// get their values, find out how many are not in order
			auto color = getColor(storage[k].color);

			if (card_val >= max_card || color == prev_color) {	// if card above the previous one is lower or the same value, or its color is the same as previous one, its out of order
				out_of_order++;
			}
			max_card = card_val;
			prev_color = color;
		}
	}

	return cards_out_of_home + out_of_order;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	std::unordered_map<std::string, std::pair<std::string, std::vector<SearchAction>>> parents;	// has to be like this, doesnt work without container for SearchAction

	// open in the form of an inverse priority queue (lowest on top), constant search time (cuz's lowest on top), somehow worse insert performance
	std::priority_queue<std::pair<double, SearchState>, std::vector<std::pair<double, SearchState>>, std::greater<std::pair<double,SearchState>>> open;	// min-heap priority queue
	open.push(std::make_pair(0.0, init_state));	// first 

	// closed
	std::unordered_set<std::string> closed;
	// open set
	std::unordered_set<std::string> open_set;
	// distance tracking from beginning
	std::unordered_map<std::string, int> distances;
	distances[stateString(init_state)] = 0;

	while (!open.empty()) {

		SearchState lowest(open.top().second);	// current lowest
		open.pop(); //remove the lowest, its been saved already
		open_set.erase(stateString(lowest));
		auto actions = lowest.actions();	// actions possible from the first state in queue
		auto distance = distances[stateString(lowest)];
		insertStringUnordered(&closed, lowest);

		if (tooMuchMemory(mem_limit_)) {
			break;
		}

		for (size_t i = 0; i < actions.size(); i++) {
			auto action = actions[i];
			auto new_state(action.execute(lowest));	// find new states derived from the original

			// instead of calling a function, comparing it here results in ~5% improvement
			if (tooMuchMemory(mem_limit_)) {
				return {};
			}

			if (!inUnorderedSet(closed, new_state) && !inUnorderedSet(open_set, new_state)) {	// not in closed set and open set

				std::pair<std::string,std::pair<std::string,SearchAction>> tmp = std::make_pair(stateString(new_state),std::make_pair(stateString(lowest),action));
				std::vector<SearchAction> actionVector {action};
				parents[stateString(new_state)] = std::make_pair(stateString(lowest), actionVector);

				auto new_dist = distance+1;
				distances[stateString(new_state)] = new_dist;	// insert new distance as its parents distance +1

				if (new_state.isFinal()) {
					return getPath(init_state, parents, tmp);
				}

				auto score = new_dist + compute_heuristic(new_state, *heuristic_);
				
				open.push(std::make_pair(score, new_state));	// push new state to the priority queue
				insertStringUnordered(&open_set, new_state);
			}
		}
	}
	return {};
}
