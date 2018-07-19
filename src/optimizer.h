#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <functional>
#include <queue>
#include <set>
#include <vector>

template<class state>
state optimize_greedy(
	state initial_state,
	const std::function<bool(const state&)>& feasible,
	const std::function<double(const state&)>& fitness,
	const std::function<std::vector<state>(const state&)>& adjacents,
	long max_steps = -1
) {
	if (!feasible(initial_state)) throw std::exception("Initial state is not feasible");

	long steps = 0;
	auto s = initial_state;
	while (true) {
		// Generate all adjacents
		std::vector<state> adjs = adjacents(s);

		// Filter out unfeasible adjacent states
		for (auto it = adjs.begin(); it != adjs.end();) {
			if (!feasible(*it)) adjs.erase(it);
			else ++it;
		}

		// Edge case: no adjacents
		if (adjs.size() == 0) return s;

		// Find state with highest fitness
		auto s_fitness = fitness(s);
		auto next_best_state = *adjs.begin();
		auto next_best_fitness = fitness(*adjs.begin());
		for (const auto& adj : adjs) {
			auto f = fitness(adj);
			if (f > next_best_fitness) {
				next_best_state = adj;
				next_best_fitness = f;
			}
		}

		// Local minimum
		if (s_fitness > next_best_fitness) {
			return s;
		}
		else {
			s = next_best_state;
		}

		// Max number of steps, to avoid infinite ascent
		if (max_steps != -1) {
			if (++steps == max_steps) return s;
		}
	}
}

template<class state>
state optimize_bfs(
	state initial_state,
	const std::function<bool(const state&)>& feasible,
	const std::function<double(const state&)>& fitness,
	const std::function<std::vector<state>(const state&)>& adjacents
) {
	if (!feasible(initial_state)) throw std::exception("Initial state is not feasible");

	auto best_state = initial_state;
	auto best_fitness = fitness(initial_state);

	std::vector<state> visited;
	std::queue<state> frontier;
	frontier.push(initial_state);
	visited.push_back(initial_state);

	while (!frontier.empty()) {
		state s = frontier.front();
		frontier.pop();
		visited.push_back(s);

		auto f = fitness(s);
		if (f > best_fitness) {
			best_state = s;
			best_fitness = f;
		}

		// Generate adjacents states, filter out unfeasible or visited
		for (const auto& adj : adjacents(s)) {
			if (feasible(adj) && 
				std::find(visited.begin(), visited.end(), adj) == visited.end() 
				) {
				frontier.push(adj);
				visited.push_back(adj);
			}
		}
	}
	return best_state;
}

#endif // OPTIMIZER_H