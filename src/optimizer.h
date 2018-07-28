#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <functional>
#include <queue>
#include <set>
#include <vector>

#define CHECK_FEASIBLE if (!feasible(initial_state)) throw std::exception("Initial state is not feasible");

template<class state>
state optimize_greedy(
	state initial_state,
	const std::function<bool(const state&)>& feasible,
	const std::function<double(const state&)>& fitness,
	const std::function<std::vector<state>(const state&)>& adjacents,
	long max_steps = -1
) {
	CHECK_FEASIBLE;

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
	CHECK_FEASIBLE;

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

template<class state>
state optimize_local_beam(
	state initial_state,
	const std::function<bool(const state&)>& feasible,
	const std::function<double(const state&)>& fitness,
	const std::function<std::vector<state>(const state&)>& adjacents,
	unsigned max_beam_size = 100
) {
	if (!feasible(initial_state)) throw std::exception("Initial state is not feasible");
	if (max_beam_size < 1) throw std::exception("Invalid beam size");

	struct state_comparator {
		std::function<double(const state&)> f;

		state_comparator(const std::function<double(const state&)>& _f) {
			f = _f;
		}

		bool operator()(const state& lhs, const state& rhs) {
			return f(lhs) < f(rhs);
		}
	};

	state_comparator state_cmp(fitness);
	std::priority_queue<state, std::vector<state>, state_comparator> frontier(state_cmp);
	std::vector<state> visited;

	frontier.emplace(initial_state);
	visited.push_back(initial_state);
	
	auto best_state = initial_state;
	auto best_fitness = fitness(initial_state);
	while (!frontier.empty()) {
		std::priority_queue<state, std::vector<state>, state_comparator> adjs(state_cmp);

		// Generate all feasible adjacents
		while (!frontier.empty()) {
			auto s = frontier.top();
			frontier.pop();

			for (const auto& adj : adjacents(s)) {
				if (std::find(visited.begin(), visited.end(), adj) == visited.end()) {
					if (feasible(adj)) {
						adjs.emplace(adj);
						visited.push_back(adj);
					}
				}
			}
		}

		// No adjacents
		if (adjs.empty()) break;
		
		auto best_adj = adjs.top();
		if (fitness(best_adj) > fitness(best_state)) {
			best_state = best_adj;
			best_fitness = fitness(best_adj);
		}

		auto max_capacity = max_beam_size;
		while (max_capacity-- && !adjs.empty()) {
			frontier.emplace(adjs.top());
			adjs.pop();
		}
	}

	return best_state;
}

template<class state>
state optimize_simulated_annealing(
	state initial_state,
	const std::function<bool(const state&)>& feasible,
	const std::function<double(const state&)>& fitness,
	const std::function<std::vector<state>(const state&)>& adjacents,
	unsigned max_iterations = UINT32_MAX
) {
	CHECK_FEASIBLE;

	auto s = initial_state;
	for (auto i = 0; i < max_iterations; i++) {
		auto adjs = adjacents(s);
		adjs.erase(
			std::remove_if(adjs.begin(), adjs.end(), [&](const auto& _s) {return !feasible(_s);}),
			adjs.end()
		);
		if (adjs.size() == 0) return s;
		auto adj = adjs[rand() % adjs.size()];
		
		auto delta_fitness = fitness(s) - fitness(adj);
		if (delta_fitness < 0.f) s = adj;
		else {
			auto t = (max_iterations - i) / float(max_iterations);
			auto p = expf(-delta_fitness/t);
			if (float(rand()) / float(RAND_MAX) <= p) s = adj;
		}
	}
	return s;
}

#endif // OPTIMIZER_H