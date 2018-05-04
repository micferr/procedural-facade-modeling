#ifndef GRAMMAR_H
#define GRAMMAR_H

#include <vector>
#include <map>
#include <queue>

#include "node.h"

namespace yb {

	using std::string;
	using std::vector;
	using std::map;
	using std::queue;
	using std::pair;

	template<typename T>
	struct production_rule {
		T from;
		std::vector<T> to;

		// This rule should be chosen, when deriving 'from', with probability
		// weight / (sum_{pr in production_rules with pr.from = this.from} pr.weight)
		float weight;
	};

	template<typename T>
	class grammar {
		T S;
		map<T, vector<production_rule<T>>> prods;

	public:
		grammar(const T& S) : S(S) {}

		/**
		 * Adds a new production rule
		 */
		void add_rule(const production_rule<T>& rule) {
			prods[rule.from].push_back(rule);
		}

		void add_rule(const T& from, const vector<T>& to, float weight = 1.f) {
			add_rule({ from,to,weight });
		}

		/**
		 * Adds multiple production rules
		 */
		void add_rules(const vector<production_rule<T>>& rules) {
			for (const auto& rule : rules) add_rule(rule);
		}
		
		void add_rules(const T& L, const vector<vector<T>>& R) {
			for (const auto& r : R) {
				add_rule({ L, r, 1.f });
			}
		}

		void add_rules(const T& L, const vector<pair<vector<T>, float>>& productions) {
			for (const auto& prod : productions) {
				add_rule({ L, prod.first, prod.second });
			}
		}

		// Whether the symbol is terminal, i.e. it is not the left side
		// of any production rule
		bool is_terminal(const T& value) {
			return prods.count(value) == 0;
		}

		// Whether the symbol is variable for the grammar, i.e. it is
		// the left side of a production rule
		bool is_variable(const T& value) {
			return !is_terminal(value);
		}

		/**
		 * Returns a random word from the grammar by using
		 * leftmost derivation.
		 * The word symbols are the leaves of the returned tree, in the
		 * order determined by a DFS, minus variables V involved in a
		 * V -> <empty> production.
		 *
		 * TODO: multithreaded expansion
		 */
		node<T>* produce() {
			node<T>* root = new node<T>(S);
			queue<node<T>*> nodes;
			nodes.push(root);
			while (!nodes.empty()) {
				node<T>* n = nodes.front();
				nodes.pop();

				if (is_variable(n->value)) {
					auto& prod = prods.find(n->value);
					auto& prod_rules = (*prod).second;
					auto& substitution = prod_rules[rand() % prod_rules.size()];
					for (auto& child_value : substitution.to) {
						n->add_child(child_value);
						nodes.push(n->children.back());
					}
				}
			}
			return root;
		}
	};
}

#endif // GRAMMAR_H
