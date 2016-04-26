//
//  docopt_private.h
//  docopt
//
//  Created by Jared Grubb on 2013-11-04.
//  Copyright (c) 2013 Jared Grubb. All rights reserved.
//

#ifndef docopt_docopt_private_h
#define docopt_docopt_private_h

#include <vector>
#include <memory>
#include <unordered_set>

#include "docopt_value.h"

namespace docopt {
	
	class Pattern;
	class LeafPattern;
	
	using PatternList = std::vector<std::shared_ptr<Pattern>>;
	
	// Utility to use Pattern types in std hash-containers
	struct PatternHasher {
		template <typename P>
		size_t operator()(std::shared_ptr<P> const& pattern) const {
			return pattern->hash();
		}
		template <typename P>
		size_t operator()(P const* pattern) const {
			return pattern->hash();
		}
		template <typename P>
		size_t operator()(P const& pattern) const {
			return pattern.hash();
		}
	};
	
	// Utility to use 'hash' as the equality operator as well in std containers
	struct PatternPointerEquality {
		template <typename P1, typename P2>
		bool operator()(std::shared_ptr<P1> const& p1, std::shared_ptr<P2> const& p2) const {
			return p1->hash()==p2->hash();
		}
		template <typename P1, typename P2>
		bool operator()(P1 const* p1, P2 const* p2) const {
			return p1->hash()==p2->hash();
		}
	};
	
	// A hash-set that uniques by hash value
	using UniquePatternSet = std::unordered_set<std::shared_ptr<Pattern>, PatternHasher, PatternPointerEquality>;

	
	class Pattern {
	public:
		// flatten out children, stopping descent when the given filter returns 'true'
		virtual std::vector<Pattern*> flat(bool (*filter)(Pattern const*)) = 0;
		
		// flatten out all children into a list of LeafPattern objects
		virtual void collect_leaves(std::vector<LeafPattern*>&) = 0;
		
		// flatten out all children into a list of LeafPattern objects
		std::vector<LeafPattern*> leaves();
		
		// Attempt to find something in 'left' that matches this pattern's spec, and if so, move it to 'collected'
		virtual bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const = 0;
		
		virtual std::string const& name() const = 0;
		
		virtual bool hasValue() const { return false; }
		
		virtual size_t hash() const = 0;

		virtual ~Pattern() = default;
	};
	
	class LeafPattern
	: public Pattern {
	public:
		LeafPattern(std::string name, value v = {})
		: fName(std::move(name)),
		  fValue(std::move(v))
		{}
		
		virtual std::vector<Pattern*> flat(bool (*filter)(Pattern const*)) override {
			if (filter(this)) {
				return { this };
			}
			return {};
		}
		
		virtual void collect_leaves(std::vector<LeafPattern*>& lst) override final {
			lst.push_back(this);
		}
		
		virtual bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;
		
		virtual bool hasValue() const override { return static_cast<bool>(fValue); }
		
		value const& getValue() const { return fValue; }
		void setValue(value&& v) { fValue = std::move(v); }
				
		virtual std::string const& name() const override { return fName; }
		
		virtual size_t hash() const override {
			size_t seed = typeid(*this).hash_code();
			hash_combine(seed, fName);
			hash_combine(seed, fValue);
			return seed;
		}

	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const&) const = 0;
		
	private:
		std::string fName;
		value fValue;
	};
	
	class BranchPattern
	: public Pattern {
	public:
		BranchPattern(PatternList children = {})
		: fChildren(std::move(children))
		{}
		
		Pattern& fix() {
			UniquePatternSet patterns;
			fix_identities(patterns);
			fix_repeating_arguments();
			return *this;
		}
		
		virtual std::string const& name() const override {
			throw std::runtime_error("Logic error: name() shouldnt be called on a BranchPattern");
		}
		
		virtual value const& getValue() const {
			throw std::runtime_error("Logic error: name() shouldnt be called on a BranchPattern");
		}
		
		virtual std::vector<Pattern*> flat(bool (*filter)(Pattern const*)) override {
			if (filter(this)) {
				return {this};
			}
			
			std::vector<Pattern*> ret;
			for(auto& child : fChildren) {
				auto sublist = child->flat(filter);
				ret.insert(ret.end(), sublist.begin(), sublist.end());
			}
			return ret;
		}
		
		virtual void collect_leaves(std::vector<LeafPattern*>& lst) override final {
			for(auto& child : fChildren) {
				child->collect_leaves(lst);
			}
		}
		
		void setChildren(PatternList children) {
			fChildren = std::move(children);
		}
		
		PatternList const& children() const { return fChildren; }
		
		virtual void fix_identities(UniquePatternSet& patterns) {
			for(auto& child : fChildren) {
				// this will fix up all its children, if needed
				if (auto bp = dynamic_cast<BranchPattern*>(child.get())) {
					bp->fix_identities(patterns);
				}
				
				// then we try to add it to the list
				auto inserted = patterns.insert(child);
				if (!inserted.second) {
					// already there? then reuse the existing shared_ptr for that thing
					child = *inserted.first;
				}
			}
		}
		
		virtual size_t hash() const override {
			size_t seed = typeid(*this).hash_code();
			hash_combine(seed, fChildren.size());
			for(auto const& child : fChildren) {
				hash_combine(seed, child->hash());
			}
			return seed;
		}
	private:
		void fix_repeating_arguments();
		
	protected:
		PatternList fChildren;
	};
	
	class Argument
	: public LeafPattern {
	public:
		using LeafPattern::LeafPattern;
		
	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const& left) const override;
	};
	
	class Command : public Argument {
	public:
		Command(std::string name, value v = value{false})
		: Argument(std::move(name), std::move(v))
		{}
		
	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const& left) const override;
	};
	
	class Option final
	: public LeafPattern
	{
	public:
		static Option parse(std::string const& option_description);
		
		Option(std::string shortOption,
		       std::string longOption,
		       int argcount = 0,
		       value v = value{false})
		: LeafPattern(longOption.empty() ? shortOption : longOption,
			      std::move(v)),
		  fShortOption(std::move(shortOption)),
		  fLongOption(std::move(longOption)),
		  fArgcount(argcount)
		{
			// From Python:
			//   self.value = None if value is False and argcount else value
			if (argcount && v.isBool() && !v.asBool()) {
				setValue(value{});
			}
		}
		
		Option(Option const&) = default;
		Option(Option&&) = default;
		Option& operator=(Option const&) = default;
		Option& operator=(Option&&) = default;
		
		using LeafPattern::setValue;
		
		std::string const& longOption() const { return fLongOption; }
		std::string const& shortOption() const { return fShortOption; }
		int argCount() const { return fArgcount; }
		
		virtual size_t hash() const override {
			size_t seed = LeafPattern::hash();
			hash_combine(seed, fShortOption);
			hash_combine(seed, fLongOption);
			hash_combine(seed, fArgcount);
			return seed;
		}
		
	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const& left) const override;

	private:
		std::string fShortOption;
		std::string fLongOption;
		int fArgcount;
	};

	class Required : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;
		
		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;
	};

	class Optional : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;
		
		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override {
			for(auto const& pattern : fChildren) {
				pattern->match(left, collected);
			}
			return true;
		}
	};

	class OptionsShortcut : public Optional {
		using Optional::Optional;
	};

	class OneOrMore : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;
		
		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;
	};

	class Either : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;
		
		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;
	};
}

#endif
