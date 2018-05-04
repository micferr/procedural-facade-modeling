#ifndef PROB_UTILS_H
#define PROB_UTILS_H

#include <limits>
#include <ctime>

#include "yocto\yocto_gl.h"

namespace yb {

	/**
	 * Returns a random boolean; true is returned with p probability
	 */
	bool bernoulli(ygl::rng_pcg32& rng, float p) {
		if (p < 0 || p > 1) throw std::exception("Invalid probability value.");
		return ygl::next_rand1f(rng) <= p;
	}

	/**
	 * Counts the number of consecutive failures before a success of
	 * a Bernoulli random variable with success probability p.
	 * 
	 * Note that the expected runtime is O(max-min)
	 */
	int geometric(
		ygl::rng_pcg32& rng,
		float p,
		unsigned min = 0,
		unsigned max = std::numeric_limits<unsigned>::max()
	) {
		if (max < min) throw std::exception("Invalid min-max range");
		int n = min;
		while (n < max && !bernoulli(rng, p)) n++;
		return n;
	}

	/**
	 * A geometric random variable's value represents the number of failures
	 * before the first success.
	 * This function is an utility around the definition of the geometric r.v.,
	 * and returns the number of successes before the first failure (i.e. it
	 * computes the value of a geometric r.v. with parameter p' = 1-p)
	 */
	int consecutive_bernoulli_successes(
		ygl::rng_pcg32& rng,
		float p,
		unsigned min = 0,
		unsigned max = std::numeric_limits<unsigned>::max()
	) {
		return geometric(rng, 1.f - p, min, max);
	}

	/**
	 * Generates a sequence of n random booleans, each of which is a Bernoulli
	 * random variable with success probability p
	 */
	std::vector<bool> bernoulli_seq(ygl::rng_pcg32& rng, unsigned n, float p) {
		std::vector<bool> v;
		while (n--) v.emplace_back(bernoulli(rng, p));
		return v;
	}

	/**
	 * Given the expected value of a geometric random variable, 
	 * returns the success probability of each Bernoulli trial
	 */
	float bernoulli_prob_from_geometric_expected_value(float n) {
		// Expected(p) = E = (1-p)/p
		// E*p = 1-p (p != 0)
		// E*p + p = 1
		// p(E+1) = 1
		// p = 1/(E+1)
		if (n < 0.f) throw std::exception("Invalid expected value for geometric r.v.");
		return 1.f / (n + 1.f);
	}

	/**
	 * Returns a random value in [min,max) with uniform distribution
	 */
	float uniform(ygl::rng_pcg32& rng, float min, float max) {
		return ygl::next_rand1f(rng)*(max - min) + min;
	}

	/**
	 * Generates a random value with normal distribution
	 */
	float gaussian(ygl::rng_pcg32& rng, float mu, float sigma) {
		// Box-Muller transform's polar form.
		// See
		//     G.E.P. Box, M.E. Muller
		//     "A Note on the Generation of Random Normal Deviates", 
		//     The Annals of Mathematical Statistics (1958), Vol. 29, No. 2 pp. 610–611
		// TODO?:
		//     Box-Muller suffers from tail-truncation.
		//     Methods such as Inverse CDF, Ziggurat and Ratio of uniforms
		//     may be better

		// This a simple implementation from the original Box and Muller's paper
		// as no better license-able version was found

		float u1 = ygl::next_rand1f(rng);
		float u2 = ygl::next_rand1f(rng);
		
		float x1 = sqrt(-2.f*log(u1))*cos(2.f*pi*u2);
		return x1 * sigma + mu;
	}

	/**
	 * Returns a random int in [0, weights.size()), with integer i
	 * having a probability of being chosen of weights[i]/(sum_j weights[j])
	 */
	int random_weighted(ygl::rng_pcg32& rng, const std::vector<float>& weights) {
		float weights_total = 0.f;
		for (auto w : weights) weights_total += w;
		float r = ygl::next_rand1f(rng, 0.f, weights_total);
		int which = 0;
		while (r > weights[which]) {
			r -= weights[which];
			which++;
		}
		return which;
	}

	/**
	 * Choose a random element from a vector
	 */
	template<typename T>
	T choose_random(ygl::rng_pcg32& rng, const std::vector<T>& v) {
		return v[ygl::next_rand1i(rng, v.size())];
	}

	/**
	 * Randomly picks an element from a vector; element v[i] is chosen
	 * with probability weights[i]/(sum_j weights[j])
	 */
	template<typename T>
	T choose_random_weighted(
		ygl::rng_pcg32& rng,
		const std::vector<T>& v,
		const std::vector<float> weights
	) {
		if (v.size() != weights.size()) {
			throw std::exception("v and weights must have equal size");
		}
		if (v.size() == 0) {
			throw std::exception("Must pick from at least one element");
		}
		return v[random_weighted(rng, weights)];
	}

	/**
	 * Returns several subvectors of the original vector.
	 *
	 * - keep_prob: the probability that a generic element appears in a substring
	 * - continue_prob: the probability that, given that a generic element appears
	 *       in a substring and its predecessor also does so, it is placed in the
	 *       same substring as its predecessor and not in a new substring
	 *
	 * Let n be the number of generated substrings, and let seqs_i be the i-th 
	 * generated substring of seq.
	 * Then:
	 * - i != j => intersection(seqs_i,seqs_j) = {}
	 * - i < j => seqs_i precedes seqs_j in seq
	 * - U_i seqs_i <= seq (in particular, it is possible that U_i seqs_i != seq)
	 */
	template<typename T>
	std::vector<std::vector<T>> random_substrings(
		ygl::rng_pcg32& rng,
		const std::vector<T>& seq,
		float keep_prob,
		float continue_prob,
		int skip_after_sequence_end = 0
	) {
		std::vector<std::vector<T>> seqs;
		std::vector<T> s;
		for (int i = 0; i < seq.size(); i++) {
			if (bernoulli(rng, keep_prob)) {
				if (s.size() == 0 || bernoulli(rng, continue_prob)) {
					s.push_back(seq[i]);
				}
				else {
					seqs.push_back(s);
					s.clear();
				}
			}
			else {
				if (s.size() != 0) {
					seqs.push_back(s);
					s.clear();
					i += skip_after_sequence_end;
				}
			}
		}
		if (s.size() > 0) seqs.push_back(s); // Last sequence
		return seqs;
	}
}

#endif // PROB_UTILS_H