//
//  docopt.cpp
//  docopt
//
//  Created by Jared Grubb on 2013-11-03.
//  Copyright (c) 2013 Jared Grubb. All rights reserved.
//

#include "docopt.h"
#include "docopt_util.h"
#include "docopt_private.h"

#include "docopt_value.h"

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <string>
#include <regex>
#include <iostream>
#include <cassert>
#include <cstddef>

using namespace docopt;

DocoptExitHelp::DocoptExitHelp()
: std::runtime_error("Docopt --help argument encountered")
{}

DocoptExitVersion::DocoptExitVersion()
: std::runtime_error("Docopt --version argument encountered")
{}

const char* value::kindAsString(Kind kind)
{
	switch (kind) {
		case Kind::Empty: return "empty";
		case Kind::Bool: return "bool";
		case Kind::Long: return "long";
		case Kind::String: return "string";
		case Kind::StringList: return "string-list";
	}
	return "unknown";
}

void value::throwIfNotKind(Kind expected) const
{
	if (kind == expected)
		return;

	std::string error = "Illegal cast to ";
	error += kindAsString(expected);
	error += "; type is actually ";
	error += kindAsString(kind);
	throw std::runtime_error(std::move(error));
}

std::ostream& docopt::operator<<(std::ostream& os, value const& val)
{
	if (val.isBool()) {
		bool b = val.asBool();
		os << (b ? "true" : "false");
	} else if (val.isLong()) {
		long v = val.asLong();
		os << v;
	} else if (val.isString()) {
		std::string const& str = val.asString();
		os << '"' << str << '"';
	} else if (val.isStringList()) {
		auto const& list = val.asStringList();
		os << "[";
		bool first = true;
		for(auto const& el : list) {
			if (first) {
				first = false;
			} else {
				os << ", ";
			}
			os << '"' << el << '"';
		}
		os << "]";
	} else {
		os << "null";
	}
	return os;
}

#pragma mark -
#pragma mark Pattern types

std::vector<LeafPattern*> Pattern::leaves() {
	std::vector<LeafPattern*> ret;
	collect_leaves(ret);
	return ret;
}

bool Required::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const
{
	auto l = left;
	auto c = collected;

	for(auto const& pattern : fChildren) {
		bool ret = pattern->match(l, c);
		if (!ret) {
			// leave (left, collected) untouched
			return false;
		}
	}

	left = std::move(l);
	collected = std::move(c);
	return true;
}

bool LeafPattern::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const
{
	auto match = single_match(left);
	if (!match.second) {
		return false;
	}

	left.erase(left.begin()+static_cast<std::ptrdiff_t>(match.first));

	auto same_name = std::find_if(collected.begin(), collected.end(), [&](std::shared_ptr<LeafPattern> const& p) {
		return p->name()==name();
	});
	if (getValue().isLong()) {
		long val = 1;
		if (same_name == collected.end()) {
			collected.push_back(match.second);
			match.second->setValue(value{val});
		} else if ((**same_name).getValue().isLong()) {
			val += (**same_name).getValue().asLong();
			(**same_name).setValue(value{val});
		} else {
			(**same_name).setValue(value{val});
		}
	} else if (getValue().isStringList()) {
		std::vector<std::string> val;
		if (match.second->getValue().isString()) {
			val.push_back(match.second->getValue().asString());
		} else if (match.second->getValue().isStringList()) {
			val = match.second->getValue().asStringList();
		} else {
			/// cant be!?
		}

		if (same_name == collected.end()) {
			collected.push_back(match.second);
			match.second->setValue(value{val});
		} else if ((**same_name).getValue().isStringList()) {
			std::vector<std::string> const& list = (**same_name).getValue().asStringList();
			val.insert(val.begin(), list.begin(), list.end());
			(**same_name).setValue(value{val});
		} else {
			(**same_name).setValue(value{val});
		}
	} else {
		collected.push_back(match.second);
	}
	return true;
}

Option Option::parse(std::string const& option_description)
{
	std::string shortOption, longOption;
	int argcount = 0;
	value val { false };

	auto double_space = option_description.find("  ");
	auto options_end = option_description.end();
	if (double_space != std::string::npos) {
		options_end = option_description.begin() + static_cast<std::ptrdiff_t>(double_space);
	}

	static const std::regex pattern {"(-{1,2})?(.*?)([,= ]|$)"};
	for(std::sregex_iterator i {option_description.begin(), options_end, pattern, std::regex_constants::match_not_null},
	       e{};
	    i != e;
	    ++i)
	{
		std::smatch const& match = *i;
		if (match[1].matched) { // [1] is optional.
			if (match[1].length()==1) {
					shortOption = "-" + match[2].str();
			} else {
					longOption =  "--" + match[2].str();
			}
		} else if (match[2].length() > 0) { // [2] always matches.
			std::string m = match[2];
			argcount = 1;
		} else {
			// delimeter
		}

		if (match[3].length() == 0) { // [3] always matches.
			// Hit end of string. For some reason 'match_not_null' will let us match empty
			// at the end, and then we'll spin in an infinite loop. So, if we hit an empty
			// match, we know we must be at the end.
			break;
		}
	}

	if (argcount) {
		std::smatch match;
		if (std::regex_search(options_end, option_description.end(),
				      match,
				      std::regex{"\\[default: (.*)\\]", std::regex::icase}))
		{
			val = match[1].str();
		}
	}

	return {std::move(shortOption),
		std::move(longOption),
		argcount,
		std::move(val)};
}

bool OneOrMore::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const
{
	assert(fChildren.size() == 1);

	auto l = left;
	auto c = collected;

	bool matched = true;
	size_t times = 0;

	decltype(l) l_;
	bool firstLoop = true;

	while (matched) {
		// could it be that something didn't match but changed l or c?
		matched = fChildren[0]->match(l, c);

		if (matched)
			++times;

		if (firstLoop) {
			firstLoop = false;
		} else if (l == l_) {
			break;
		}

		l_ = l;
	}

	if (times == 0) {
		return false;
	}

	left = std::move(l);
	collected = std::move(c);
	return true;
}

bool Either::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const
{
	using Outcome = std::pair<PatternList, std::vector<std::shared_ptr<LeafPattern>>>;

	std::vector<Outcome> outcomes;

	for(auto const& pattern : fChildren) {
		// need a copy so we apply the same one for every iteration
		auto l = left;
		auto c = collected;
		bool matched = pattern->match(l, c);
		if (matched) {
			outcomes.emplace_back(std::move(l), std::move(c));
		}
	}

	auto min = std::min_element(outcomes.begin(), outcomes.end(), [](Outcome const& o1, Outcome const& o2) {
		return o1.first.size() < o2.first.size();
	});

	if (min == outcomes.end()) {
		// (left, collected) unchanged
		return false;
	}

	std::tie(left, collected) = std::move(*min);
	return true;
}

std::pair<size_t, std::shared_ptr<LeafPattern>> Argument::single_match(PatternList const& left) const
{
	std::pair<size_t, std::shared_ptr<LeafPattern>> ret {};

	for(size_t i = 0, size = left.size(); i < size; ++i)
	{
		auto arg = dynamic_cast<Argument const*>(left[i].get());
		if (arg) {
			ret.first = i;
			ret.second = std::make_shared<Argument>(name(), arg->getValue());
			break;
		}
	}

	return ret;
}

std::pair<size_t, std::shared_ptr<LeafPattern>> Command::single_match(PatternList const& left) const
{
	std::pair<size_t, std::shared_ptr<LeafPattern>> ret {};

	for(size_t i = 0, size = left.size(); i < size; ++i)
	{
		auto arg = dynamic_cast<Argument const*>(left[i].get());
		if (arg) {
			if (name() == arg->getValue()) {
				ret.first = i;
				ret.second = std::make_shared<Command>(name(), value{true});
			}
			break;
		}
	}

	return ret;
}

std::pair<size_t, std::shared_ptr<LeafPattern>> Option::single_match(PatternList const& left) const
{
	std::pair<size_t, std::shared_ptr<LeafPattern>> ret {};

	for(size_t i = 0, size = left.size(); i < size; ++i)
	{
		auto leaf = std::dynamic_pointer_cast<LeafPattern>(left[i]);
		if (leaf && name() == leaf->name()) {
			ret.first = i;
			ret.second = leaf;
			break;
		}
	}

	return ret;
}

#pragma mark -
#pragma mark Parsing stuff

static std::vector<PatternList> transform(PatternList pattern);

void BranchPattern::fix_repeating_arguments()
{
	std::vector<PatternList> either = transform(children());
	for(auto const& group : either) {
		// use multiset to help identify duplicate entries
		std::unordered_multiset<std::shared_ptr<Pattern>, PatternHasher> group_set {group.begin(), group.end()};
		for(auto const& e : group_set) {
			if (group_set.count(e) == 1)
				continue;

			LeafPattern* leaf = dynamic_cast<LeafPattern*>(e.get());
			if (!leaf) continue;

			bool ensureList = false;
			bool ensureInt = false;

			if (dynamic_cast<Command*>(leaf)) {
				ensureInt = true;
			} else if (dynamic_cast<Argument*>(leaf)) {
				ensureList = true;
			} else if (Option* o = dynamic_cast<Option*>(leaf)) {
				if (o->argCount()) {
					ensureList = true;
				} else {
					ensureInt = true;
				}
			}

			if (ensureList) {
				std::vector<std::string> newValue;
				if (leaf->getValue().isString()) {
					newValue = split(leaf->getValue().asString());
				}
				if (!leaf->getValue().isStringList()) {
					leaf->setValue(value{newValue});
				}
			} else if (ensureInt) {
				leaf->setValue(value{0});
			}
		}
	}
}

static std::vector<PatternList> transform(PatternList pattern)
{
	std::vector<PatternList> result;

	std::vector<PatternList> groups;
	groups.emplace_back(std::move(pattern));

	while(!groups.empty()) {
		// pop off the first element
		auto children = std::move(groups[0]);
		groups.erase(groups.begin());

		// find the first branch node in the list
		auto child_iter = std::find_if(children.begin(), children.end(), [](std::shared_ptr<Pattern> const& p) {
			return dynamic_cast<BranchPattern const*>(p.get());
		});

		// no branch nodes left : expansion is complete for this grouping
		if (child_iter == children.end()) {
			result.emplace_back(std::move(children));
			continue;
		}

		// pop the child from the list
		auto child = std::move(*child_iter);
		children.erase(child_iter);

		// expand the branch in the appropriate way
		if (Either* either = dynamic_cast<Either*>(child.get())) {
			// "[e] + children" for each child 'e' in Either
			for(auto const& eitherChild : either->children()) {
				PatternList group = { eitherChild };
				group.insert(group.end(), children.begin(), children.end());

				groups.emplace_back(std::move(group));
			}
		} else if (OneOrMore* oneOrMore = dynamic_cast<OneOrMore*>(child.get())) {
			// child.children * 2 + children
			auto const& subchildren = oneOrMore->children();
			PatternList group = subchildren;
			group.insert(group.end(), subchildren.begin(), subchildren.end());
			group.insert(group.end(), children.begin(), children.end());

			groups.emplace_back(std::move(group));
		} else { // Required, Optional, OptionsShortcut
			BranchPattern* branch = dynamic_cast<BranchPattern*>(child.get());

			// child.children + children
			PatternList group = branch->children();
			group.insert(group.end(), children.begin(), children.end());

			groups.emplace_back(std::move(group));
		}
	}

	return result;
}

class Tokens {
public:
	Tokens(std::vector<std::string> tokens, bool isParsingArgv = true)
	: fTokens(std::move(tokens)),
	  fIsParsingArgv(isParsingArgv)
	{}

	explicit operator bool() const {
		return fIndex < fTokens.size();
	}

	static Tokens from_pattern(std::string const& source) {
		static const std::regex re_separators {
			"(?:\\s*)" // any spaces (non-matching subgroup)
			"("
			"[\\[\\]\\(\\)\\|]" // one character of brackets or parens or pipe character
			"|"
			"\\.\\.\\."  // elipsis
			")" };

		static const std::regex re_strings {
			"(?:\\s*)" // any spaces (non-matching subgroup)
			"("
			"\\S*<.*?>"  // strings, but make sure to keep "< >" strings together
			"|"
			"[^<>\\s]+"     // string without <>
			")" };

		// We do two stages of regex matching. The '[]()' and '...' are strong delimeters
		// and need to be split out anywhere they occur (even at the end of a token). We
		// first split on those, and then parse the stuff between them to find the string
		// tokens. This is a little harder than the python version, since they have regex.split
		// and we dont have anything like that.

		std::vector<std::string> tokens;
		std::for_each(std::sregex_iterator{ source.begin(), source.end(), re_separators },
			      std::sregex_iterator{},
			      [&](std::smatch const& match)
			      {
				      // handle anything before the separator (this is the "stuff" between the delimeters)
				      if (match.prefix().matched) {
					      std::for_each(std::sregex_iterator{match.prefix().first, match.prefix().second, re_strings},
							    std::sregex_iterator{},
							    [&](std::smatch const& m)
							    {
								    tokens.push_back(m[1].str());
							    });
				      }

				      // handle the delimter token itself
				      if (match[1].matched) {
					      tokens.push_back(match[1].str());
				      }
			      });

		return Tokens(tokens, false);
	}

	std::string const& current() const {
		if (*this)
			return fTokens[fIndex];

		static std::string const empty;
		return empty;
	}

	std::string the_rest() const {
		if (!*this)
			return {};
		return join(fTokens.begin()+static_cast<std::ptrdiff_t>(fIndex),
			    fTokens.end(),
			    " ");
	}

	std::string pop() {
		return std::move(fTokens.at(fIndex++));
	}

	bool isParsingArgv() const { return fIsParsingArgv; }

	struct OptionError : std::runtime_error { using runtime_error::runtime_error; };

private:
	std::vector<std::string> fTokens;
	size_t fIndex = 0;
	bool fIsParsingArgv;
};

// Get all instances of 'T' from the pattern
template <typename T>
std::vector<T*> flat_filter(Pattern& pattern) {
	std::vector<Pattern*> flattened = pattern.flat([](Pattern const* p) -> bool {
		return dynamic_cast<T const*>(p) != nullptr;
	});

	// now, we're guaranteed to have T*'s, so just use static_cast
	std::vector<T*> ret;
	std::transform(flattened.begin(), flattened.end(), std::back_inserter(ret), [](Pattern* p) {
		return static_cast<T*>(p);
	});
	return ret;
}

static std::vector<std::string> parse_section(std::string const& name, std::string const& source) {
	// ECMAScript regex only has "?=" for a non-matching lookahead. In order to make sure we always have
	// a newline to anchor our matching, we have to avoid matching the final newline of each grouping.
	// Therefore, our regex is adjusted from the docopt Python one to use ?= to match the newlines before
	// the following lines, rather than after.
	std::regex const re_section_pattern {
		"(?:^|\\n)"  // anchored at a linebreak (or start of string)
		"("
		   "[^\\n]*" + name + "[^\\n]*(?=\\n?)" // a line that contains the name
		   "(?:\\n[ \\t].*?(?=\\n|$))*"         // followed by any number of lines that are indented
		")",
		std::regex::icase
	};

	std::vector<std::string> ret;
	std::for_each(std::sregex_iterator(source.begin(), source.end(), re_section_pattern),
		      std::sregex_iterator(),
		      [&](std::smatch const& match)
	{
		ret.push_back(trim(match[1].str()));
	});

	return ret;
}

static bool is_argument_spec(std::string const& token) {
	if (token.empty())
		return false;

	if (token[0]=='<' && token[token.size()-1]=='>')
		return true;

	if (std::all_of(token.begin(), token.end(), &::isupper))
		return true;

	return false;
}

template <typename I>
std::vector<std::string> longOptions(I iter, I end) {
	std::vector<std::string> ret;
	std::transform(iter, end,
		       std::back_inserter(ret),
		       [](typename I::reference opt) { return opt->longOption(); });
	return ret;
}

static PatternList parse_long(Tokens& tokens, std::vector<Option>& options)
{
	// long ::= '--' chars [ ( ' ' | '=' ) chars ] ;
	std::string longOpt, equal;
	value val;
	std::tie(longOpt, equal, val) = partition(tokens.pop(), "=");

	assert(starts_with(longOpt, "--"));

	if (equal.empty()) {
		val = value{};
	}

	// detect with options match this long option
	std::vector<Option const*> similar;
	for(auto const& option : options) {
		if (option.longOption()==longOpt)
			similar.push_back(&option);
	}

	// maybe allow similar options that match by prefix
	if (tokens.isParsingArgv() && similar.empty()) {
		for(auto const& option : options) {
			if (option.longOption().empty())
				continue;
			if (starts_with(option.longOption(), longOpt))
				similar.push_back(&option);
		}
	}

	PatternList ret;

	if (similar.size() > 1) { // might be simply specified ambiguously 2+ times?
		std::vector<std::string> prefixes = longOptions(similar.begin(), similar.end());
		std::string error = "'" + longOpt + "' is not a unique prefix: ";
		error.append(join(prefixes.begin(), prefixes.end(), ", "));
		throw Tokens::OptionError(std::move(error));
	} else if (similar.empty()) {
		int argcount = equal.empty() ? 0 : 1;
		options.emplace_back("", longOpt, argcount);

		auto o = std::make_shared<Option>(options.back());
		if (tokens.isParsingArgv()) {
			o->setValue(argcount ? value{val} : value{true});
		}
		ret.push_back(o);
	} else {
		auto o = std::make_shared<Option>(*similar[0]);
		if (o->argCount() == 0) {
			if (val) {
				std::string error = o->longOption() + " must not have an argument";
				throw Tokens::OptionError(std::move(error));
			}
		} else {
			if (!val) {
				auto const& token = tokens.current();
				if (token.empty() || token=="--") {
					std::string error = o->longOption() + " requires an argument";
					throw Tokens::OptionError(std::move(error));
				}
				val = tokens.pop();
			}
		}
		if (tokens.isParsingArgv()) {
			o->setValue(val ? std::move(val) : value{true});
		}
		ret.push_back(o);
	}

	return ret;
}

static PatternList parse_short(Tokens& tokens, std::vector<Option>& options)
{
	// shorts ::= '-' ( chars )* [ [ ' ' ] chars ] ;

	auto token = tokens.pop();

	assert(starts_with(token, "-"));
	assert(!starts_with(token, "--"));

	auto i = token.begin();
	++i; // skip the leading '-'

	PatternList ret;
	while (i != token.end()) {
		std::string shortOpt = { '-', *i };
		++i;

		std::vector<Option const*> similar;
		for(auto const& option : options) {
			if (option.shortOption()==shortOpt)
				similar.push_back(&option);
		}

		if (similar.size() > 1) {
			std::string error = shortOpt + " is specified ambiguously "
			+ std::to_string(similar.size()) + " times";
			throw Tokens::OptionError(std::move(error));
		} else if (similar.empty()) {
			options.emplace_back(shortOpt, "", 0);

			auto o = std::make_shared<Option>(options.back());
			if (tokens.isParsingArgv()) {
				o->setValue(value{true});
			}
			ret.push_back(o);
		} else {
			auto o = std::make_shared<Option>(*similar[0]);
			value val;
			if (o->argCount()) {
				if (i == token.end()) {
					// consume the next token
					auto const& ttoken = tokens.current();
					if (ttoken.empty() || ttoken=="--") {
						std::string error = shortOpt + " requires an argument";
						throw Tokens::OptionError(std::move(error));
					}
					val = tokens.pop();
				} else {
					// consume all the rest
					val = std::string{i, token.end()};
					i = token.end();
				}
			}

			if (tokens.isParsingArgv()) {
				o->setValue(val ? std::move(val) : value{true});
			}
			ret.push_back(o);
		}
	}

	return ret;
}

static PatternList parse_expr(Tokens& tokens, std::vector<Option>& options);

static PatternList parse_atom(Tokens& tokens, std::vector<Option>& options)
{
	// atom ::= '(' expr ')' | '[' expr ']' | 'options'
	//             | long | shorts | argument | command ;

	std::string const& token = tokens.current();

	PatternList ret;

	if (token == "[") {
		tokens.pop();

		auto expr = parse_expr(tokens, options);

		auto trailing = tokens.pop();
		if (trailing != "]") {
			throw DocoptLanguageError("Mismatched '['");
		}

		ret.emplace_back(std::make_shared<Optional>(std::move(expr)));
	} else if (token=="(") {
		tokens.pop();

		auto expr = parse_expr(tokens, options);

		auto trailing = tokens.pop();
		if (trailing != ")") {
			throw DocoptLanguageError("Mismatched '('");
		}

		ret.emplace_back(std::make_shared<Required>(std::move(expr)));
	} else if (token == "options") {
		tokens.pop();
		ret.emplace_back(std::make_shared<OptionsShortcut>());
	} else if (starts_with(token, "--") && token != "--") {
		ret = parse_long(tokens, options);
	} else if (starts_with(token, "-") && token != "-" && token != "--") {
		ret = parse_short(tokens, options);
	} else if (is_argument_spec(token)) {
		ret.emplace_back(std::make_shared<Argument>(tokens.pop()));
	} else {
		ret.emplace_back(std::make_shared<Command>(tokens.pop()));
	}

	return ret;
}

static PatternList parse_seq(Tokens& tokens, std::vector<Option>& options)
{
	// seq ::= ( atom [ '...' ] )* ;"""

	PatternList ret;

	while (tokens) {
		auto const& token = tokens.current();

		if (token=="]" || token==")" || token=="|")
			break;

		auto atom = parse_atom(tokens, options);
		if (tokens.current() == "...") {
			ret.emplace_back(std::make_shared<OneOrMore>(std::move(atom)));
			tokens.pop();
		} else {
			std::move(atom.begin(), atom.end(), std::back_inserter(ret));
		}
	}

	return ret;
}

static std::shared_ptr<Pattern> maybe_collapse_to_required(PatternList&& seq)
{
	if (seq.size()==1) {
		return std::move(seq[0]);
	}
	return std::make_shared<Required>(std::move(seq));
}

static std::shared_ptr<Pattern> maybe_collapse_to_either(PatternList&& seq)
{
	if (seq.size()==1) {
		return std::move(seq[0]);
	}
	return std::make_shared<Either>(std::move(seq));
}

PatternList parse_expr(Tokens& tokens, std::vector<Option>& options)
{
	// expr ::= seq ( '|' seq )* ;

	auto seq = parse_seq(tokens, options);

	if (tokens.current() != "|")
		return seq;

	PatternList ret;
	ret.emplace_back(maybe_collapse_to_required(std::move(seq)));

	while (tokens.current() == "|") {
		tokens.pop();
		seq = parse_seq(tokens, options);
		ret.emplace_back(maybe_collapse_to_required(std::move(seq)));
	}

	return { maybe_collapse_to_either(std::move(ret)) };
}

static Required parse_pattern(std::string const& source, std::vector<Option>& options)
{
	auto tokens = Tokens::from_pattern(source);
	auto result = parse_expr(tokens, options);

	if (tokens)
		throw DocoptLanguageError("Unexpected ending: '" + tokens.the_rest() + "'");

	assert(result.size() == 1  &&  "top level is always one big");
	return Required{ std::move(result) };
}


static std::string formal_usage(std::string const& section) {
	std::string ret = "(";

	auto i = section.find(':')+1;  // skip past "usage:"
	auto parts = split(section, i);
	for(size_t ii = 1; ii < parts.size(); ++ii) {
		if (parts[ii] == parts[0]) {
			ret += " ) | (";
		} else {
			ret.push_back(' ');
			ret += parts[ii];
		}
	}

	ret += " )";
	return ret;
}

static PatternList parse_argv(Tokens tokens, std::vector<Option>& options, bool options_first)
{
	// Parse command-line argument vector.
	//
	// If options_first:
	//    argv ::= [ long | shorts ]* [ argument ]* [ '--' [ argument ]* ] ;
	// else:
	//    argv ::= [ long | shorts | argument ]* [ '--' [ argument ]* ] ;

	PatternList ret;
	while (tokens) {
		auto const& token = tokens.current();

		if (token=="--") {
			// option list is done; convert all the rest to arguments
			while (tokens) {
				ret.emplace_back(std::make_shared<Argument>("", tokens.pop()));
			}
		} else if (starts_with(token, "--")) {
			auto&& parsed = parse_long(tokens, options);
			std::move(parsed.begin(), parsed.end(), std::back_inserter(ret));
		} else if (token[0]=='-' && token != "-") {
			auto&& parsed = parse_short(tokens, options);
			std::move(parsed.begin(), parsed.end(), std::back_inserter(ret));
		} else if (options_first) {
			// option list is done; convert all the rest to arguments
			while (tokens) {
				ret.emplace_back(std::make_shared<Argument>("", tokens.pop()));
			}
		} else {
			ret.emplace_back(std::make_shared<Argument>("", tokens.pop()));
		}
	}

	return ret;
}

static std::vector<Option> parse_defaults(std::string const& doc) {
	// This pattern is a bit more complex than the python docopt one due to lack of
	// re.split. Effectively, it grabs any line with leading whitespace and then a
	// hyphen; it stops grabbing when it hits another line that also looks like that.
	static std::regex const pattern {
		"(?:^|\\n)[ \\t]*"  // a new line with leading whitespace
		"(-(.|\\n)*?)"      // a hyphen, and then grab everything it can...
		"(?=\\n[ \\t]*-|$)" //  .. until it hits another new line with space and a hyphen
	};

	std::vector<Option> defaults;

	for(auto s : parse_section("options:", doc)) {
		s.erase(s.begin(), s.begin()+static_cast<std::ptrdiff_t>(s.find(':'))+1); // get rid of "options:"

		std::for_each(std::sregex_iterator{ s.begin(), s.end(), pattern },
			      std::sregex_iterator{},
			      [&](std::smatch const& m)
		{
			std::string opt = m[1].str();

			if (starts_with(opt, "-")) {
				defaults.emplace_back(Option::parse(opt));
			}
		});
	}

	return defaults;
}

static bool isOptionSet(PatternList const& options, std::string const& opt1, std::string const& opt2 = "") {
	return std::any_of(options.begin(), options.end(), [&](std::shared_ptr<Pattern const> const& opt) -> bool {
		auto const& name = opt->name();
		if (name==opt1 || (!opt2.empty() && name==opt2)) {
			return opt->hasValue();
		}
		return false;
	});
}

static void extras(bool help, bool version, PatternList const& options) {
	if (help && isOptionSet(options, "-h", "--help")) {
		throw DocoptExitHelp();
	}

	if (version && isOptionSet(options, "--version")) {
		throw DocoptExitVersion();
	}
}

// Parse the doc string and generate the Pattern tree
static std::pair<Required, std::vector<Option>> create_pattern_tree(std::string const& doc)
{
	auto usage_sections = parse_section("usage:", doc);
	if (usage_sections.empty()) {
		throw DocoptLanguageError("'usage:' (case-insensitive) not found.");
	}
	if (usage_sections.size() > 1) {
		throw DocoptLanguageError("More than one 'usage:' (case-insensitive).");
	}

	std::vector<Option> options = parse_defaults(doc);
	Required pattern = parse_pattern(formal_usage(usage_sections[0]), options);

	std::vector<Option const*> pattern_options = flat_filter<Option const>(pattern);

	using UniqueOptions = std::unordered_set<Option const*, PatternHasher, PatternPointerEquality>;
	UniqueOptions const uniq_pattern_options { pattern_options.begin(), pattern_options.end() };

	// Fix up any "[options]" shortcuts with the actual option tree
	for(auto& options_shortcut : flat_filter<OptionsShortcut>(pattern)) {
		std::vector<Option> doc_options = parse_defaults(doc);

		// set(doc_options) - set(pattern_options)
		UniqueOptions uniq_doc_options;
		for(auto const& opt : doc_options) {
			if (uniq_pattern_options.count(&opt))
				continue;
			uniq_doc_options.insert(&opt);
		}

		// turn into shared_ptr's and set as children
		PatternList children;
		std::transform(uniq_doc_options.begin(), uniq_doc_options.end(),
			       std::back_inserter(children), [](Option const* opt) {
				       return std::make_shared<Option>(*opt);
			       });
		options_shortcut->setChildren(std::move(children));
	}

	return { std::move(pattern), std::move(options) };
}

std::map<std::string, value>
docopt::docopt_parse(std::string const& doc,
		     std::vector<std::string> const& argv,
		     bool help,
		     bool version,
		     bool options_first)
{
	Required pattern;
	std::vector<Option> options;
	try {
		std::tie(pattern, options) = create_pattern_tree(doc);
	} catch (Tokens::OptionError const& error) {
		throw DocoptLanguageError(error.what());
	}

	PatternList argv_patterns;
	try {
		argv_patterns = parse_argv(Tokens(argv), options, options_first);
	} catch (Tokens::OptionError const& error) {
		throw DocoptArgumentError(error.what());
	}

	extras(help, version, argv_patterns);

	std::vector<std::shared_ptr<LeafPattern>> collected;
	bool matched = pattern.fix().match(argv_patterns, collected);
	if (matched && argv_patterns.empty()) {
		std::map<std::string, value> ret;

		// (a.name, a.value) for a in (pattern.flat() + collected)
		for (auto* p : pattern.leaves()) {
			ret[p->name()] = p->getValue();
		}

		for (auto const& p : collected) {
			ret[p->name()] = p->getValue();
		}

		return ret;
	}

	if (matched) {
		std::string leftover = join(argv.begin(), argv.end(), ", ");
		throw DocoptArgumentError("Unexpected argument: " + leftover);
	}

	throw DocoptArgumentError("Arguments did not match expected patterns"); // BLEH. Bad error.
}

std::map<std::string, value>
docopt::docopt(std::string const& doc,
	       std::vector<std::string> const& argv,
	       bool help,
	       std::string const& version,
	       bool options_first) noexcept
{
	try {
		return docopt_parse(doc, argv, help, !version.empty(), options_first);
	} catch (DocoptExitHelp const&) {
		std::cout << doc << std::endl;
		std::exit(0);
	} catch (DocoptExitVersion const&) {
		std::cout << version << std::endl;
		std::exit(0);
	} catch (DocoptLanguageError const& error) {
		std::cerr << "Docopt usage string could not be parsed" << std::endl;
		std::cerr << error.what() << std::endl;
		std::exit(-1);
	} catch (DocoptArgumentError const& error) {
		std::cerr << error.what();
		std::cout << std::endl;
		std::cout << doc << std::endl;
		std::exit(-1);
	} /* Any other exception is unexpected: let std::terminate grab it */
}
