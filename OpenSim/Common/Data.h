#ifndef OPENSIM_DATA_H_
#define OPENSIM_DATA_H_

/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Data.h                                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Nabeel Allana                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/** @file
* This file defines the OpenSim data input / output interface.
* It defines abstract classes for implimenting custom data readers
* and writers, as well as OpenSim components designed to hook up
* data readers and writers to other OpenSim components.
*/

// INCLUDES
#include <OpenSim/Common/osimCommonDLL.h>
#include "OpenSim/Common/Object.h"
#include "OpenSim/Common/ComponentConnector.h"
#include "OpenSim/Common/ComponentOutput.h"
#include "OpenSim/Common/Component.h"
#include "Simbody.h"

#include <functional>
#include <string>
#include <sstream>
#include <memory>
#include <type_traits>
#include <exception>
#include <map>
#include <queue>

namespace OpenSim {
	/* 
		A dictionary is a serializable packet of information.
		It is designed so that any OpenSim serializable object
		can be inserted by name, and then retrieved again later.
	  
		Use the put<T>(std::string name, T value) to insert an
		object into the dictionary. Once inserted, it can be
		retrieved again by name using the get<T>(std::string name)
		method. 
	 */
	struct OSIMCOMMON_API Dictionary : public Object
	{
	public:
		template<class T>
		void put(std::string name, T value)
		{
			if (!hasProperty(name))
				addProperty(name, "", value);
			else
			{
				Property<T>& prop = updProperty<T>(getPropertyIndex(name));
				prop.setValue(value);
			}
		}
		
		template<class T>
		T& get(std::string name)
		{
			if (!hasProperty(name))
				Exception("Item with name '" + name + "' not found in dictionary", __FILE__, __LINE__);

			Property<T>& prop = updProperty<T>(getPropertyIndex(name));
			return prop.updValue();
		}
	};

	/*
		A data iterator is a representation of single element within a data set,
		and embeds logic within it to advance to subsequent elements, or to
		go to previous elements within the same set. The data iterator is an 
		abstract class which is implimented by a data reader and allows the data
		reader to outline the method by which data is accessed within it.

		For example, a Matlab Data Reader may impliment a custom data iterator
		that is designed to iterator over certain columns of a certain variable
		that is supplied by the user. This data iterator would contain the variable
		name within it, along with the desired column references and current index.
		
		Note: A DataIterator may represent a ficticious point within the data.
		In, for example, a streaming data reader, it may represent the end point. In 
		this case, the "end" of the data has no real meaning, and therefore a custom
		implimentation of the DataIterator could embedd that information within it.
		In this case, a custom search and / or interpolate function may be needed
		for successfull use of the data interface. See DataInputChannel documentation
		and source code before implimenting a streaming data reader.
	*/
	template<class Ty>
	class OSIMCOMMON_API DataIterator
	{
	public:
		typedef DataIterator<Ty> self_type;
		typedef std::shared_ptr<self_type> ptr;
		typedef SimTK::Real Tx;

		virtual ~DataIterator() { }

		// Next point operator. This operator advances the current data point by one,
		// returning the old data point back. For example if x is a data point, in
		// a = x++, a would be the un-advanced value of x, and x would be advanced by one
		// element.
		//
		// Note: this should be an extrememly light operation. No data access should be
		// preformed here - only an internal representation of the data location should
		// be changed. This operation is allowed to return a data point that is in an
		// invalid state (i.e. no more data), so long as the bool() operator is correctly
		// designed to detect such state.
		virtual ptr operator++() = 0;

		// Previous point operator. This operator goes back one element in the data set,
		// returning the old data point back. For example if x is a data point, in
		// a = x--, a would be the old value of x, and x would be one element prior.
		//
		// Note: this should be an extrememly light operation. No data access should be
		// preformed here - only an internal representation of the data location should
		// be changed. This operation is allowed to return a data point that is in an
		// invalid state (i.e. no more data), so long as the bool() operator is correctly
		// designed to detect such state.
		virtual ptr operator--() = 0;

		// Step by count operator. This operator advances a specified offset in the 
		// data set, returning the old data point back. For example if x is a data point,
		// a = x+5 would be the old value of x, and x would be 5 elements ahead. Should
		// work for negative values as well.
		//
		// Note: this should be an extrememly light operation. No data access should be
		// preformed here - only an internal representation of the data location should
		// be changed. This operation is allowed to return a data point that is in an
		// invalid state (i.e. no more data), so long as the bool() operator is correctly
		// designed to detect such state.
		virtual ptr operator+(int ct) = 0;

		ptr operator-(int ct)
		{
			return *this + (-1 * ct);
		}

		// Returns number of points between two data points. This is used by some
		// interpolation and search algorithms to manipulate the data provided by the
		// data set.
		virtual const int operator-(self_type& other) = 0;

		// Provides the dependent value of the data from the server. This is one of
		// two places where data should be read from the server. If data read is 
		// an expensive operation (i.e. web / sql), it should be cached in this object 
		// for future retrievals.
		virtual const Ty& y() = 0;

		// Provides the independent value of the data from the server. This is one of 
		// two places where data should be read from the server. If data read is 
		// an expensive operation (i.e. web / sql), it should be cached in this object 
		// for future retrievals.
		virtual const Tx& x() const = 0;

		// Compares two data points against each other. Internal representations should
		// be compared, not the actual data.
		virtual bool operator==(const self_type& rhs) = 0;
		virtual bool operator!=(const self_type& rhs) = 0;

		// Checks to see if the data point object is in a valid state for reading from.
		// This is there since some data servers may define certain "data points" as 
		// imaginary placeholders to aid in serving data conveniently (i.e. a streaming
		// data server defining a "point" as infinity to represent the end point).
		virtual operator bool() const = 0;
	};

	/*
		A container to hold a pair of DataIterator objects. The pair would represent
		the start and end of a desired range of data within the data set.

		See DataIterator for details.
	*/
	template<class Ty>
	struct OSIMCOMMON_API DataIteratorRange
	{
		typedef typename DataIterator<Ty>::ptr TDataIterator;

		DataIteratorRange(TDataIterator _begin, TDataIterator _end)
		{
			begin = _begin;
			end = _end;
		}

		DataIteratorRange() {}

		TDataIterator begin;
		TDataIterator end;
	};
	
	class OSIMCOMMON_API DataManipulators
	{
	public:
		// Takes the two known data points surrounding the desired unknown point
		// and returns a point within them using the following:
		//		y = y_0 + (x - x_0) * (y_1 - y_0) / (x_1 - x_0)
		template<class Tx, class Ty>
		static Ty linear_interpolator(typename DataIterator<Ty>::ptr lbound,
			typename DataIterator<Ty>::ptr ubound,
			const Tx& x)
		{
			// todo: add lots of error checking

			if (!(lbound && ubound))
				throw Exception("invalid data range to interpolate over", __FILE__, __LINE__);

			if (*lbound == *ubound)
				return lbound->y();

			return lbound->y() + (x - lbound->x()) * (ubound->y() - lbound->y()) / (ubound->x() - lbound->x());
		}

		// Always returns the lower bound
		template<class Tx, class Ty>
		static Ty step_interpolator(typename DataIterator<Ty>::ptr lbound,
			typename DataIterator<Ty>::ptr ubound,
			const Tx& x)
		{
			// todo: add lots of error checking

			if (!(lbound))
				throw Exception("invalid data range to interpolate over", __FILE__, __LINE__);

			return lbound->y();
		}

		// Finds the closest two data points to the given x-value using
		// a binary search algorithm. Algorithm provides a O(log(n)) time
		// provided that the data point object impliments addition, negation,
		// distance and divide by constant operators.
		template<class Tx, class Ty>
		static void binary_search(typename DataIterator<Ty>::ptr& lbound,
			typename DataIterator<Ty>::ptr& ubound,
			const Tx& x)
		{
			// todo: add lots of error checking

			int len = *lbound - *ubound;

			if (*lbound - *ubound <= 1)
				return;

			typename DataIterator<Ty>::ptr mid = *lbound + ((*ubound - *lbound) / 2);

			Tx mid_x = mid->x();

			if (mid_x == x)
				lbound = ubound = mid;
			else if (mid_x > x)
				ubound = mid;
			else
				lbound = mid;

			binary_search<Tx, Ty>(lbound, ubound, x);
		}
	};

	/*
		The DataReadAdapter class defines the interface for an adapter designed to
		read data into OpenSim. A data reader (a class that has a base class of
		DataReadAdapter) must respond to a query (see Dictionary) with a DataIteratorRange
		(see DataIterator and DataIteratorRange) that spans the data requested by the
		query. For example, a Matlab data reader would take in a query with the variable
		name of interest, along with the column numbers of interest and return an
		iterator capable of traversing that data. A data adapter is used in conjunction with
		a DataInputChannel, which then serves the data as a function of time to the
		component its output is connected to.

		Additionally, a data reader may add items to its metadata dictionary. The
		metadata can be anything the data reader would like to provide as constants to the
		consumer. For example, a motion capture data adapter may add a metadata entry
		containing a vector defining the origin. See Dictionary for details.

		Note that any data reader implimenting this interface is expected to output its
		data in the metric system as users of data readers will expect to use them
		interchangeably.
	*/
	template<class Ty>
	class OSIMCOMMON_API DataReadAdapter : Component
	{
		OpenSim_DECLARE_ABSTRACT_OBJECT_T(DataReadAdapter, Ty, Component);
	public:
		typedef SimTK::Real Tx;

		virtual DataIteratorRange<Ty> query(Dictionary& query) = 0;

		Dictionary& metadata()
		{
			return _meta;
		}
	protected:
		Dictionary _meta;
	};

	/*
		The DataWriteAdapter interface defines the interface for how data may be
		written out from OpenSim. A data writer (a class that impliments the DataWriteAdapter
		interface) must impliment an open, write and close method. A data writer is used in
		conjunction with a DataOutputChannel, which recieves data from its component input
		port, buffers the data while sorting it by time, and then flushes the data to
		the data writer. Therefore, the data coming into the data writer is gaurenteed to
		be ordered by time (x), but not in real time. Note that if order of data is not
		important but rather real time data is prefered (i.e. if streaming out over the
		network), the requireRealTime() method should be overridden. See requireRealTime()
		and write() for details.
	*/
	template<class Ty>
	class OSIMCOMMON_API DataWriteAdapter : Component
	{
		OpenSim_DECLARE_ABSTRACT_OBJECT_T(DataWriteAdapter, Ty, Component);
	public:
		typedef SimTK::Real Tx;

		// Function is called when the data writer should
		// close its output streams as data will no longer
		// be provided to it.
		virtual void close() = 0;

		// Handle incomming data. Data will always come in
		// ascending order of x unless the requireRealTime
		// method is overloaded. If requireRealTime is set,
		// ordering is not guaranteed.
		virtual void write(Tx& x, Ty& y) = 0;

		// All initialization logic of data writer should go
		// on here (i.e. open output streams). 
		virtual void open() = 0;

		// See: write()
		virtual bool requireRealTime()
		{ return false; }

		Dictionary& metadata()
		{ return _meta; }
	private:
		Dictionary _meta;
	};

	/*
		A DataOutputChannel is an OpenSim Component that allows data two be written out from
		OpenSim. The destination of the data is determined by the DataWriteAdapter that is
		connected to the 'adapter' structural connection of this component. To use this
		component, instantiate an instance of the DataOutputChannel, along with an instance
		of the desired data writer (see DataWriteAdapter). Connect the 'adapter' input of the
		DataOutputChannel to the data writer and the 'input' input of the DataOutputChannel
		to the output of the component from which the data should be read from.
	*/
	template<class Ty>
	class OSIMCOMMON_API DataOutputChannel : public Component
	{
		OpenSim_DECLARE_CONCRETE_OBJECT_T(DataOutputChannel, Ty, Component);
	public:

		typedef SimTK::Real Tx;
		typedef DataWriteAdapter<Ty> TDataWriteAdapter;

		DataOutputChannel()
		{
			setAuthors("Nabeel Allana");
		}

		~DataOutputChannel()
		{
			flush();
		}

		void close()
		{
			flush();
			_writer->close();
		}
		
		Dictionary& metadata()
		{
			return _writer->metadata();
		}
	private:

		TDataWriteAdapter* _writer;

		struct DataBlock
		{
			Tx x;
			Ty y;
		};

		struct DataBlockComparator
		{
			// default comparator
			bool operator()(const DataBlock& rb, const DataBlock& lb)
			{
				return rb.x > lb.x;
			}
		};

		std::priority_queue<DataBlock, std::vector<DataBlock>, DataBlockComparator> queue;

		void insert(Tx& x, Ty& y)
		{
			if (_writer->requireRealTime())
				_writer->write(x, y);
			else
			{
				DataBlock blk;
				blk.x = x;
				blk.y = y;
				queue.push(blk);
			}
		}

		void flush()
		{
			while (!queue.empty())
			{
				DataBlock& blk = queue.top();
				_writer->write(blk.x, blk.y);
				queue.pop();
			}
		}

		void constructInputs() OVERRIDE_11
		{
			constructInput<Ty>("input", SimTK::Stage::LowestValid);

			constructStructuralConnector<DataWriteAdapter<Ty>>("adapter");
		}

		void realizeReport(const SimTK::State& state) const OVERRIDE_11
		{
			Tx& x = state.getTime();

			insert(x, getInputValue<Ty>(state, "input"));
		}
	protected:

		void connect(Component& root) OVERRIDE_11
		{
			Super::connect(root);

			DataWriteAdapter<Ty>& adapter = getConnector<DataWriteAdapter<Ty>>("adapter");

			_writer = &adapter;

			_writer->open();
		}
	};

	/*
		A DataInputChannel is an OpenSim Component that allows data to be read into OpenSim.
		The source of the data is determined by the DataReadAdapter instance the channel is
		configured to, along with the Query the channel is using. The DataReadAdapter serves
		data bounds to the DataInputChannel (see DataIterator). To connect a component to
		external data, instantiate a DataInputChannel and data reader (see DataReadAdapter)
		for where the data is coming from. Connect the data reader to the "adapter" structural
		connector of the DataInputChannel. Set the Query to the data reader in the DataInputChannel
		(this is adapter specific, so consult the documentation for the adapter in use to
		see what - if any - properties they desire) using the setQuery method. Finally, wire
		the "output" output of the DataInputChannel to the component that will be recieveing the
		data from this channel.
	*/
	template<class Ty>
	class OSIMCOMMON_API DataInputChannel : public Component
	{
		OpenSim_DECLARE_CONCRETE_OBJECT_T(DataInputChannel, Ty, Component);
	public:
		OpenSim_DECLARE_PROPERTY(interpolation_method, std::string, 
			"The interpolation method this channel is configured to use");

		OpenSim_DECLARE_PROPERTY(query, Dictionary, 
			"The query this channel is configured to use");

		enum class InterpolationFunction
		{
			LINEAR,
			SPLINE,
			STEP
		};

		typedef SimTK::Real Tx;

		// shorthands
		typedef SimTK::State TState;
		typedef typename DataIterator<Ty>::ptr TIterator;
		typedef DataIteratorRange<Ty> TDataIteratorRange;

		// what the prototype of the search function needs to look like
		typedef std::function<void(TIterator&, TIterator&, const Tx&)> TSearchFunction;

		// what the prototype of the interpolate function needs to look like
		typedef std::function<Ty(TIterator&, TIterator&, const Tx&)> TInterpolateFunction;

		// construct a data source using an DataIterator pair object
		DataInputChannel(Dictionary& _query = Dictionary())
		{
			_init();
		}

		Dictionary& metadata()
		{
			return reader.metadata();
		}

		// set the interpolation method
		void setInterpolator(TInterpolateFunction fn)
		{
			set_interpolation_method("lambda");
			int_fn = fn;
		}

		// set the search method
		void setSearch(TSearchFunction fn)
		{
			search_fn = fn;
		}

		void setInterpolator(InterpolationFunction fn)
		{
			switch (fn)
			{
			case InterpolationFunction::LINEAR:
				int_fn = DataManipulators::linear_interpolator<Tx, Ty>;
				set_interpolation_method("linear");
				break;
			case InterpolationFunction::SPLINE:
				set_interpolation_method("spline");
				break;
			case InterpolationFunction::STEP:
				int_fn = DataManipulators::step_interpolator<Tx, Ty>;
				set_interpolation_method("step");
				break;
			};
		}

		void setQuery(Dictionary& dict)
		{
			set_query(dict);
		}
	
	protected:

		void connect(Component& root) OVERRIDE_11
		{
			Super::connect(root);

			DataReadAdapter<Ty>& adapter = getConnector<DataReadAdapter<Ty>>("adapter");

			Dictionary& query = get_query();

			range = adapter.query(query);

			std::string& int_method = get_interpolation_method();

			switch (int_method)
			{
			case "linear":
				int_fn = DataManipulators::linear_interpolator<Tx, Ty>;
				break;
			case "spline":
				break;
			case "step":
				int_fn = DataManipulators::step_interpolator<Tx, Ty>;
				break;
			};
		}

	private:

		// gets the output value for the given state
		Ty getValueAt(TState& state)
		{
			Tx& x = state.getTime();

			// copy our begin/end DataIterators
			TIterator hBound = range.begin;
			TIterator lBound = range.end;

			// search in those bounds for our x-value
			search_fn(hBound, lBound, x);

			// interpolate within those bounds and return
			// result

			Ty result = int_fn(hBound, lBound, x);

			return result;
		}

		void _init()
		{
			setAuthors("Nabeel Allana");

			// set our default search and interpolation algorightms
			search_fn = TSearchFunction(&DataManipulators::binary_search<Tx, Ty>);
			int_fn = TInterpolateFunction(&DataManipulators::step_interpolator<Tx, Ty>);

			constructProperty_interpolation_method("step");
			constructProperty_query(Dictionary());

			constructInfrastructure();
		}

		void constructOutputs() OVERRIDE_11
		{
			constructOutput<Ty>(
				"output",
				std::bind(&DataInputChannel<Ty>::getValueAt, this, std::placeholders::_1), 
				SimTK::Stage::Time
			);
		}

		void constructInputs() OVERRIDE_11
		{
			constructStructuralConnector<DataReadAdapter<Ty>>("adapter");
		}

		// the search function
		TSearchFunction search_fn;
		// the interpolation function
		TInterpolateFunction int_fn;

		TDataIteratorRange range;
	};
}

#endif