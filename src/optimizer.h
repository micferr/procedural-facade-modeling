#ifndef OPTIMIZERS_H
#define OPTIMIZERS_H

#include <ctime>
#include <exception>
#include <functional>
#include <queue>
#include <random>
#include <set>
#include <vector>

// Utility class for random numbers
struct random_generator {
	std::mt19937 rand_engine; // Public for simplicity

	random_generator() {
		rand_engine.seed(time(nullptr));
	}
	int rand(int min, int max) {
		return std::uniform_int_distribution<int>(min, max)(rand_engine);
	};
	float rand(float min, float max) {
		return std::uniform_real_distribution<float>(min, max)(rand_engine);
	}
};

enum param_type {
	INTEGER, REAL
};

class parameter {
	std::string parname;
	int imin, imax, ival; // For an integer parameter
	float rmin, rmax, rval; // For a real parameter
	param_type type;
public:
	// Constructors
	parameter(const std::string& name, int min, int max, int val) {
		if (!(min <= val && val <= max)) {
			throw new std::runtime_error("Invalid parameter values");
		}
		parname = name;
		imin = min;
		imax = max;
		ival = val;
		type = INTEGER;
	}
	parameter(const std::string& name, int min, int max) : parameter(name, min, max, (min + max) / 2) {}
	parameter(const std::string& name, float min, float max, float val) {
		if (!(min <= val && val <= max)) {
			throw new std::runtime_error("Invalid parameter values");
		}
		parname = name;
		rmin = min;
		rmax = max;
		rval = val;
		type = REAL;
	}
	parameter(const std::string& name, float min, float max) : parameter(name, min, max, (min + max) / 2.f) {}

	const std::string& get_name() const {
		return parname;
	}

	// Returns the bounds of the integer parameter
	const std::pair<int,int> get_ibounds() const {
		if (type == INTEGER) {
			return { imin,imax };
		}
		else {
			throw new std::runtime_error("Invalid parameter type");
		}
	}
	int get_imin() const {
		return get_ibounds().first;
	}
	int get_imax() const {
		return get_ibounds().second;
	}

	// Returns the bounds of the real parameter
	const std::pair<float, float> get_rbounds() const {
		if (type == REAL) {
			return { rmin, rmax };
		}
		else {
			throw new std::runtime_error("Invalid parameter type");
		}
	}
	float get_rmin() const {
		return get_rbounds().first;
	}
	float get_rmax() const {
		return get_rbounds().second;
	}

	param_type get_type() const {
		return type;
	}

	bool is_integer() const {
		return type == INTEGER;
	}
	bool is_real() const {
		return !is_integer();
	}

	int get_ival() const {
		if (is_integer()) {
			return ival;
		}
		else {
			throw new std::runtime_error("Invalid parameter type");
		}
	}
	float get_rval() const {
		if (is_real()) {
			return rval;
		}
		else {
			throw new std::runtime_error("Invalid parameter type");
		}
	}

	void set_ival(int value) {
		if (is_integer() && imin <= value && value <= imax) {
			ival = value;
		}
		else {
			throw new std::runtime_error("Invalid parameter type or value");
		}
	}
	void set_rval(float value) {
		if (is_real() && rmin <= value && value <= rmax) {
			rval = value;
		}
		else {
			throw new std::runtime_error("Invalid parameter type or value");
		}
	}

	void set_random(random_generator& rg) {
		if (is_integer()) ival = rg.rand(imin, imax);
		else rval = rg.rand(rmin, rmax);
	}
};

class optimization_problem {
	// The constraints this problem is subject to (empty if unconstrained).
	// A constraint is expressed through a boolean function of the parameters,
	// returning true if the assignment is feasible
	std::vector<std::function<bool(const std::vector<parameter>&)>> constraints;

	// The objectives (one or more) of the problem, utility functions to be maximized
	std::vector<std::function<float(const std::vector<parameter>&)>> objectives;

	// The problem's parameters
	std::vector<parameter> params;
public:
	
	// Checks the kind of problem
	bool is_integer_problem() const {
		if (params.size() == 0) return false;
		for (const auto& p : params) {
			if (p.is_real()) return false;
		}
		return true;
	};
	bool is_continuous_problem() const {
		if (params.size() == 0) return false;
		for (const auto& p : params) {
			if (p.is_integer()) return false;
		}
		return true;
	};
	bool is_mixed_integer_problem() const {
		if (params.size() == 0) return false;
		return !(is_integer_problem() || is_continuous_problem());
	};

	bool is_single_objective() const {
		return objectives.size() == 1;
	}
	bool is_multiobjective() const {
		return objectives.size() > 1;
	}

	bool is_constrained() const {
		return constraints.size() > 0;
	}
	bool is_unconstrained() const {
		return !is_constrained();
	}

	// Checks whether the given parameter assignment is feasible
	// It is assumed that the input parameters are the ones required by the problem.
	bool is_feasible(const std::vector<parameter>& parameters) const {
		for (const auto& c : constraints) {
			if (!c(parameters)) {
				return false;
			}
		}
		return true;
	}

	// Checks whether the current parameter assignment is feasible
	bool is_feasible() const {
		return is_feasible(params);
	}

	// Returns the fitness value for a parameter assignment to this (single-objective) problem.
	// It is assumed that the input parameters are the ones required by the problem.
	float get_objective_value(const std::vector<parameter>& parameters) const {
		if (is_single_objective()) {
			return objectives[0](parameters);
		}
		else {
			throw new std::exception("Invalid problem type");
		}
	}

	// Returns the fitness value for a parameter assignment to this (multi-objective) problem.
	// It is assumed that the input parameters are the ones required by the problem.
	std::vector<float> get_objectives_values(const std::vector<parameter>& parameters) const {
		std::vector<float> values(objectives.size());
		for (auto i = 0; i < objectives.size(); i++) {
			values[i] = objectives[i](parameters);
		}
		return values;
	}

	// Returns the fitness value for a single-objective problem
	float get_objective_value() const {
		return get_objective_value(params);
	}

	// Returns the fitness values for a multi-objective problem
	std::vector<float> get_objectives_values() const {
		return get_objectives_values(params);
	}

	void add_constraint(std::function<bool(const std::vector<parameter>&)> constraint) {
		constraints.push_back(constraint);
	}

	void add_objective(std::function<float(const std::vector<parameter>&)> objective) {
		objectives.push_back(objective);
	}

	void add_parameter(const parameter& param) {
		params.push_back(param);
	}

	// Returning a const vector would be better to prevent unwanted modifications
	// of the parameter vector (e.g. removing parameters or adding new ones), 
	// however both std::vector<T>::operator[] and std::vector<T>::at() are not const
	// and thus can't be used on a const vector.
	// Good faith use is assumed
	std::vector<parameter>& get_params() {
		return params;
	}
};

// Utility functions for checking the algorithm is compatible with the problem type
void require_single_objective(const optimization_problem& problem) {
	if (!problem.is_single_objective()) {
		throw new std::runtime_error("Invalid problem type");
	}
}
void require_multiobjective(const optimization_problem& problem) {
	if (!problem.is_multiobjective()) {
		throw new std::runtime_error("Invalid problem type");
	}
}
void require_constrained(const optimization_problem& problem) {
	if (!problem.is_constrained()) {
		throw new std::runtime_error("Invalid problem type");
	}
}
void require_unconstrained(const optimization_problem& problem) {
	if (!problem.is_unconstrained()) {
		throw new std::runtime_error("Invalid problem type");
	}
}
void require_integer(const optimization_problem& problem) {
	if (!problem.is_integer_problem()) {
		throw new std::runtime_error("Invalid problem type");
	}
}
void require_continuous(const optimization_problem& problem) {
	if (!problem.is_continuous_problem()) {
		throw new std::runtime_error("Invalid problem type");
	}
}
void require_feasible(const optimization_problem& problem) {
	if (!problem.is_feasible()) {
		throw new std::runtime_error("Invalid problem type");
	}
}

/// Optimization algorithms ///
// Convention: the algorithms return true iff the final assignment of the parameters
// is feasible and the problem has a single objective function.
// For multiobjective optimization, a vector of solutions is returned instead (empty if
// no solution is found).
// TODO: Find a more uniform convention

// Tries 'tries' random assignments of the parameters and chooses the one which
// maximizes fitness.
// Only implemented as an example
bool best_random(
	optimization_problem& problem,
	unsigned tries
) {
	require_single_objective(problem);
	random_generator rg;

	std::vector<parameter> try_params = problem.get_params();
	float best_fitness = 0.f;
	bool first_iteration = true;
	while (tries--) {
		for (auto& p : try_params) p.set_random(rg);
		if (!problem.is_feasible()) continue; // Skip unfeasible assignments
		auto fitness = problem.get_objective_value(try_params);
		if (first_iteration || fitness > best_fitness) {
			first_iteration = false;
			problem.get_params() = try_params;
			best_fitness = fitness;
		}
	}
	// first_iteration is true iff no valid assignment has been found
	return !first_iteration;
}

// Hill climbing algorithm: at each step, choose the neighbour that
// maximizes the utility function
// In this implementation, a neighbour is an assignment where a parameter
// is either increased or decreased by 1 
bool hill_climbing(optimization_problem& problem) {
	require_single_objective(problem);
	require_integer(problem);
	// For a constrained problem, a feasible starting assignment is required
	if (problem.is_constrained()) {
		require_feasible(problem);
	}

	auto& params = problem.get_params();
	while (true) {
		// 0 -> first parameter decreased by one
		// 1 -> first parameter increased by one
		// 2 -> second parameter decreased by one
		// ...
		// -1 -> no feasible neighbour
		int best_neighbour_id = -1;
		float best_fitness = problem.get_objective_value(params);

		// Find the best neighbour
		for (auto i = 0; i < params.size(); i++) {
			auto& p = params[i];
			auto v = p.get_ival();

			// Decrease parameter by one
			if (p.get_imin() < v) {
				p.set_ival(v - 1);
				if (problem.is_feasible(params)) {
					auto f = problem.get_objective_value(params);
					if (f > best_fitness) {
						best_fitness = f;
						best_neighbour_id = 2 * i;
					}
				}
				p.set_ival(v);
			}

			// Increase parameter by one
			if (v < p.get_imax()) {
				p.set_ival(v + 1);
				if (problem.is_feasible(params)) {
					auto f = problem.get_objective_value(params);
					if (f > best_fitness) {
						best_fitness = f;
						best_neighbour_id = 2 * i + 1;
					}
				}
				p.set_ival(v);
			}
		}
		if (best_neighbour_id != -1) {
			auto& param = params[best_neighbour_id / 2];
			auto v = param.get_ival();
			auto delta = best_neighbour_id % 2 ? +1 : -1;
			param.set_ival(v + delta);
		}
		else {
			break;
		}
	}
	return true;
}

// Hooke and Jeeves' pattern search.
// 
// The search terminates when delta is lower than eps
bool pattern_search(
	optimization_problem& problem,
	float delta,
	float eps = 0.0001
) {
	require_single_objective(problem);
	require_continuous(problem);
	// For a constrained problem, a feasible starting assignment is required
	if (problem.is_constrained()) {
		require_feasible(problem);
	}

	auto& params = problem.get_params();
	while (true) {
		// 0 -> first parameter decreased by delta
		// 1 -> first parameter increased by delta
		// 2 -> second parameter decreased by delta
		// ...
		// -1 -> no feasible neighbour
		int best_neighbour_id = -1;
		float best_fitness = problem.get_objective_value(params);
		
		// Find the best neighbour
		for (auto i = 0; i < params.size(); i++) {
			auto& p = params[i];
			auto v = p.get_rval();

			// Decrease parameter by delta
			if (p.get_rmin() <= v-delta) {
				p.set_rval(v - delta);
				if (problem.is_feasible(params)) {
					auto f = problem.get_objective_value(params);
					if (f > best_fitness) {
						best_fitness = f;
						best_neighbour_id = 2 * i;
					}
				}
				// Restore the parameter value
				p.set_rval(v);
			}

			// Increase parameter by delta
			if (v + delta < p.get_rmax()) {
				p.set_rval(v + delta);
				if (problem.is_feasible(params)) {
					auto f = problem.get_objective_value(params);
					if (f > best_fitness) {
						best_fitness = f;
						best_neighbour_id = 2 * i + 1;
					}
				}
				p.set_rval(v);
			}
		}

		if (best_neighbour_id != -1) { // Better neighbour found
			auto& param = params[best_neighbour_id / 2];
			param.set_rval(param.get_rval() + (best_neighbour_id % 2 ? delta : -delta));
		}
		else if (delta/2.f >= eps) { // delta can be halved
			delta /= 2.f;
		}
		else { // delta too low, algorithm terminates 
			break;
		}
	}
	return true;
}

bool particle_swarm(
	optimization_problem& problem,
	float inertia,
	float cognitive_factor,
	float social_factor,
	int num_particles,
	unsigned num_iterations
) {
	if (num_particles == 0) {
		throw std::runtime_error("At least one particle is needed.");
	}

	require_single_objective(problem);
	require_continuous(problem);
	require_unconstrained(problem);

	random_generator rg;
	auto& params = problem.get_params();
	std::vector<parameter> dummy_params = params; // Used for calculations
	auto compute_fitness = [&](const std::vector<float>& v) {
		for (auto i = 0; i < params.size(); i++) dummy_params[i].set_rval(v[i]);
		return problem.get_objective_value(dummy_params);
	};

	// Utility struct to manage particles
	struct particle {
		std::vector<float> position; // The current assignment for the particle
		std::vector<float> best_position; // Best known position for this particle
		std::vector<float> velocity;

		particle(unsigned np) : position(np), best_position(np), velocity(np) {};
	};

	// Initialize num_particles random particle
	particle _p(params.size());
	std::vector<particle> particles(num_particles, _p);
	for (auto i = 0; i < num_particles; i++) {
		auto& p = particles[i];
		for (auto j = 0; j < params.size(); j++) {
			p.position[j] = rg.rand(params[j].get_rmin(), params[j].get_rmax());
			p.velocity[j] = rg.rand(
				-(params[j].get_rmax()-params[j].get_rmin()), 
				params[j].get_rmax()-params[j].get_rmin()
			);
		}
		p.best_position = p.position;
	}

	// Find global best particle
	auto global_best_position = particles[0].position;
	auto global_best_fitness = compute_fitness(global_best_position);
	for (auto i = 1; i < params.size(); i++) {
		auto fitness = compute_fitness(particles[i].position);
		if (fitness > global_best_fitness) {
			global_best_fitness = fitness;
			global_best_position = particles[i].position;
		}
	}

	for (auto i = 0; i < num_iterations; i++) {
		auto iter_best_fitness = global_best_fitness;
		auto iter_best_particle = -1;

		// Update particles
		for (auto j = 0; j < particles.size(); j++) {
			auto& part = particles[j];

			// Update velocity
			for (auto k = 0; k < params.size(); k++) {
				auto r_p = rg.rand(0.f, 1.f);
				auto r_g = rg.rand(0.f, 1.f);
				part.velocity[k] =
					inertia * part.velocity[k] +
					cognitive_factor * r_p * (part.best_position[k] - part.position[k]) +
					social_factor * r_g * (global_best_position[k] - part.position[k]);
			}
			// Update position and best individual position
			for (auto k = 0; k < params.size(); k++) {
				part.position[k] += part.velocity[k];
				
				// Force particles to stay inside search space
				if (part.position[k] > params[k].get_rmax()) {
					part.position[k] = params[k].get_rmax();
					part.velocity[k] = 0.f;
				}
				else if (part.position[k] < params[k].get_rmin()) {
					part.position[k] = params[k].get_rmin();
					part.velocity[k] = 0.f;
				}
			}
			
			// If needed, update the current particle and iteration's best positions
			auto fitness = compute_fitness(part.position);
			if (fitness > compute_fitness(part.best_position)) {
				part.best_position = part.position;
				if (fitness > iter_best_fitness) {
					iter_best_fitness = fitness;
					iter_best_particle = j;
				}
			}
		}

		// If needed, update global best
		if (iter_best_particle != -1) {
			global_best_fitness = iter_best_fitness;
			global_best_position = particles[iter_best_particle].position;
		}
	}

	for (int i = 0; i < params.size(); i++) {
		params[i].set_rval(global_best_position[i]);
	}
	return true; // A solution is always found
}

typedef std::vector<std::vector<parameter>> solutions;

// SPEA-2 algorithm, as described in 
// SPEA2: Improving the Strength Pareto Evolutionary Algorithm (Zitzler et al., 2001)
solutions spea2(
	optimization_problem& problem,
	unsigned population_size,
	unsigned archive_size,
	unsigned max_generations,
	const std::function<std::vector<parameter>(
		const std::vector<parameter>&,
		const std::vector<parameter>&
		)>& mating_operator = [](auto a, auto b) {
			// Shuffle crossover
			random_generator rg;
			std::vector<parameter> child = a;
			// About 50% parameters from each parent - chosen at random
			for (int i = 0; i < child.size(); i++) {
				if (rg.rand(0.f, 1.f) >= 0.5f) child[i].set_rval(b[i].get_rval());
			}
			return child;
		},
	const std::function<void(std::vector<parameter>&)>& mutation_operator = [](auto& params) {
		random_generator rg;
		for (auto& param : params) {
		// Change the value of each parameter by at most 10%, with a probability of 0.1
			if (rg.rand(0.f, 1.f) >= 0.9f) {
				auto min_new = std::max(param.get_rmin(), param.get_rval()*0.9f);
				auto max_new = std::min(param.get_rmax(), param.get_rval()*1.1f);
				param.set_rval(rg.rand(min_new, max_new));
			}
		}
	}
) {
	require_continuous(problem);
	require_unconstrained(problem);

	if (population_size < 1 || max_generations < 1) {
		return {};
	}

	auto& params = problem.get_params();
	random_generator rg;

	struct individual {
		std::vector<parameter> params;
		std::vector<float> objective_values;
		float fitness;

		bool dominates(const individual& ind) const {
			bool at_least_one_is_higher = false;
			for (auto i = 0; i < objective_values.size(); i++) {
				if (objective_values[i] < ind.objective_values[i]) return false;
				else if (objective_values[i] > ind.objective_values[i]) at_least_one_is_higher = true;
			}
			return at_least_one_is_higher;
		}
	};
	std::vector<individual> population, archive;

	/// Utility functions

	// Strength(i) := # individuals dominated by i
	auto strength = [&](
		const individual& ind,
		const std::vector<individual>& pop,
		const std::vector<individual>& arch
		) {
		auto pred = [&](const individual& i) {return ind.dominates(i);};
		auto cnt = std::count_if(pop.begin(), pop.end(), pred) +
			std::count_if(arch.begin(), arch.end(), pred);
		return cnt;
	};

	// RawFitness(i) := Sum of the strengths of individuals dominating i
	auto raw_fitness = [&](
		const individual& ind,
		const std::vector<individual>& pop,
		const std::vector<individual>& arch
		) {
		auto f = 0.f;
		for (const auto& i : pop) {
			if (i.dominates(ind)) f += strength(i, pop, arch);
		}
		for (const auto& i : arch) {
			if (i.dominates(ind)) f += strength(i, pop, arch);
		}
		return f;
	};

	// Used for sorting by distance, thus computing the root for the actual distance can be omitted
	auto squared_distance = [](const std::vector<parameter>& a, const std::vector<parameter>& b) {
		auto dist = 0.f;
		for (int i = 0; i < a.size(); i++) {
			auto delta = a[i].get_rval() - b[i].get_rval();
			dist += delta*delta;
		}
		return dist;
	};

	// Density(i) := distance of i's parameter vector from its k-th nearest
	// k is usually sqrt(|population| + |archive|)
	auto density = [&](
		const individual& ind,
		std::vector<individual>& pop,
		std::vector<individual>& arch
		) {
		std::vector<individual*> all;
		std::for_each(pop.begin(), pop.end(), [&](auto& i) {all.push_back(&i);});
		std::for_each(arch.begin(), arch.end(), [&](auto& i) {all.push_back(&i);});

		std::sort(all.begin(), all.end(), [&](auto i1, auto i2) {
			return
				squared_distance(ind.params, (*i1).params) <
				squared_distance(ind.params, (*i2).params);
		});

		auto k = int(sqrtf(all.size()));
		return 1.f / (2.f + sqrtf(squared_distance(ind.params, (*all[k]).params)));
	};

	auto is_dominated = [&](
		const individual& ind,
		std::vector<individual>& pop,
		std::vector<individual>& arch
		) {
		for (auto& i : pop) if (i.dominates(ind)) return true;
		for (auto& i : arch) if (i.dominates(ind)) return true;
		return false;
	};

	/// Algorithm

	// Generate population
	for (auto i = 0; i < population_size; i++) {
		individual ind;
		for (const auto& p : params) {
			ind.params.push_back(p);
			ind.params.back().set_random(rg);
		}
		ind.objective_values = problem.get_objectives_values(ind.params);
		population.push_back(ind);
	}

	for (auto t = 0; t < max_generations; t++) {
		// Find each individual's fitness as the sum of its raw fitness and 
		// density (lower is better)
		for (auto& ind : population) {
			ind.objective_values = problem.get_objectives_values(ind.params);
			ind.fitness =
				raw_fitness(ind, population, archive) +
				density(ind, population, archive);
		}
		for (auto& ind : archive) {
			ind.objective_values = problem.get_objectives_values(ind.params);
			ind.fitness =
				raw_fitness(ind, population, archive) +
				density(ind, population, archive);
		}
		std::shuffle(population.begin(), population.end(), rg.rand_engine);
		std::shuffle(archive.begin(), archive.end(), rg.rand_engine);

		// Fill archive of next iteration
		std::vector<individual> next_archive;
		// First, copy all nondominated solutions from population and archive
		std::vector<individual*> dominated;
		for (auto i = 0; i < population.size() && next_archive.size() < archive_size; i++) {
			if (!is_dominated(population[i], population, archive)) {
				next_archive.push_back(population[i]);
			}
			else {
				dominated.push_back(&population[i]);
			}
		}
		for (auto i = 0; i < archive.size() && next_archive.size() < archive_size; i++) {
			if (!is_dominated(archive[i], population, archive)) {
				next_archive.push_back(archive[i]);
			}
			else {
				dominated.push_back(&archive[i]);
			}
		}
		
		while (next_archive.size() > archive_size) {
			// Truncation operation

			// Less-than operator as described in the paper (section 3.2)
			//
			// i1 is less than i2 in the weak ordering iff either:
			// - sigma_i_k = sigma_j_k for all k
			// - Exists k s.t. sigma_i_l = sigma_j_l for 0 < l < k and sigma_i_k < sigma_j_k
			auto less_than = [&](const auto& i1, const auto& i2) {
				std::vector<individual*> dist1, dist2;
				for (auto& i : next_archive) {
					dist1.push_back(&i);
					dist2.push_back(&i);
				}
				// Sort by proximity to i1 and i2, respectively
				std::sort(dist1.begin(), dist1.end(), [&](auto& _i1, auto& _i2){
					return 
						squared_distance(i1.params, _i1->params) < 
						squared_distance(i1.params, _i2->params);
				});
				std::sort(dist1.begin(), dist1.end(), [&](auto& _i1, auto& _i2){
					return
						squared_distance(i2.params, _i1->params) <
						squared_distance(i2.params, _i1->params);
				});
				// Ignore the nearest element as it's the individual itself
				for (auto i = 1; i < dist1.size(); i++) {
					auto sigma_i = squared_distance(i1.params, dist1[i]->params);
					auto sigma_j = squared_distance(i2.params, dist2[i]->params);
					if (sigma_i < sigma_j) return true;
					else if (sigma_i > sigma_j) return false;
				}
				return true;
			};
			// Find the minimum element and delete it
			std::sort(next_archive.begin(), next_archive.end(), less_than);
			next_archive.erase(next_archive.begin());
		}
		if (next_archive.size() < archive_size) {
			// Choose the best dominated individuals
			std::sort(dominated.begin(), dominated.end(), [&](auto i1, auto i2) {
				return
					raw_fitness(*i1, population, archive) + density(*i1, population, archive) <
					raw_fitness(*i2, population, archive) + density(*i2, population, archive);
			});
			auto num_needed = archive_size - next_archive.size();
			for (auto i = 0; i < num_needed; i++) {
				next_archive.push_back(*(dominated[i]));
			}
		}

		//Tournament, mating, mutation
		population.clear();
		for (auto i = 0; i < population_size; i++) {
			unsigned parents[2];
			for (auto j = 0; j < 2; j++) {
				// Choose two random individual for each parent role, and choose the best
				// among the two with probability 0.8
				unsigned candidate_parents[2];
				for (auto k = 0; k < 2; k++) candidate_parents[k] = rg.rand(0, next_archive.size()-1);
				auto fitness0 = raw_fitness(next_archive[candidate_parents[0]], {}, next_archive);
				auto fitness1 = raw_fitness(next_archive[candidate_parents[1]], {}, next_archive);
				if (fitness0 < fitness1) {
					std::swap(candidate_parents[0], candidate_parents[1]);
				} 
				parents[j] = candidate_parents[rg.rand(0.f, 1.f) > 0.8f ? 0 : 1];
			}

			auto child_params = mating_operator(
				next_archive[parents[0]].params, next_archive[parents[1]].params
			);
			mutation_operator(child_params);
			individual child;
			child.params = child_params;
			child.objective_values = problem.get_objectives_values(child.params);
			population.push_back(child);
		}
		archive.clear();
		for (auto& i : next_archive) archive.push_back(i);
	}

	solutions sols;
	// Take all non-dominated individuals from the archive as solutions
	for (int i = 0; i < archive.size(); i++) {
		bool dominated = false;
		for (int j = 0; j < archive.size(); j++) {
			if (i == j) continue;
			if (archive[j].dominates(archive[i])) {
				dominated = true;
				break;
			}
			if (!dominated) {
				sols.push_back(archive[i].params);
			}
		}
	}
	return sols;
}

#endif // OPTMIZERS_H