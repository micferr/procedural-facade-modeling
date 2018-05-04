#ifndef NODE_H
#define NODE_H

#include <functional>
#include <vector>

namespace yb {

	/**
	 * A simple tree implementation.
	 *
	 * A node manages the memory of its children (i.e. the children are 
	 * in heap memory and deleted on the node's destruction). 
	 *
	 * TODO: Refactoring
	 */
	template<typename T>
	struct node {
		// Attributes are all public while working on a better interface.
		// Handle with care :)
		std::vector<node<T>*> children;
		T value;
		node<T>* parent;

		node(const T& value, node<T>* parent = nullptr) : value(value), parent(parent) {}

		~node() {
			for (auto c : children) delete c;
		}

		void add_child(const T& child_value) {
			children.push_back(new node<T>(child_value, this));
		}

	};

	template<typename T>
	void preorder_visit(const node<T>& n, const std::function<void(const T&)>& action) {
		action(value);
		for (auto c : children) {
			preorder_visit(*c, action);
		}
	}

	template<typename T>
	void preorder_visit(const node<T>& n, const std::function<void(const node<T>&)>& action) {
		action(n);
		for (auto c : n.children) {
			preorder_visit(*c, action);
		}
	}

	template<typename T>
	void print_tree(const node<T>& root) {
		// Using an internal lambda rather than a default argument prevents
		// external calls with level != 0 
		std::function<void(const node<T>&, int)> __print_tree = [&__print_tree](const node<T>& n, int level) {
			for (int i = 0; i < level - 1; i++) std::cout << "|   ";
			if (level > 0) std::cout << "|---";
			std::cout << n.value << "\n";
			for (auto c : n.children) {
				__print_tree(*c, level + 1);
			}
		};

		__print_tree(root, 0);
	}

	/**
	 * Returns the depth of a tree.
	 *
	 * A tree consisting of one node has depth 0
	 */
	template<typename T>
	int depth(const node<T>& root) {
		int d = 0;
		for (auto c : root.children) {
			d = std::max(d, 1 + depth(*c));
		}
		return d;
	}
}

#endif // NODE_H