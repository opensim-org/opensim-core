#ifndef OPENSIM_VTP_FILE_ADAPTER_H_
#define OPENSIM_VTP_FILE_ADAPTER_H_
/* -------------------------------------------------------------------------- *
 *                              VTPFileAdapter.h                              *
 * -------------------------------------------------------------------------- *
 * Author(s): Colin Smith                                                     *
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


#include "OpenSim/Common/FileAdapter.h"
#include "SimTKmath.h"
#include "osimJAMDLL.h"

namespace OpenSim {

    class OSIMJAM_API VTPFileAdapter : public FileAdapter {
    //class VTPFileAdapter : public FileAdapter {
    public:

       VTPFileAdapter();
       VTPFileAdapter(const VTPFileAdapter&) = default;
       VTPFileAdapter(VTPFileAdapter&&) = default;
       VTPFileAdapter& operator=(const VTPFileAdapter&) = default;
       VTPFileAdapter& operator=(VTPFileAdapter&&) = default;
       ~VTPFileAdapter() = default;

       VTPFileAdapter* clone() const override;


	   /**
	   @param[in] vertices
			A matrix [nVertices x nTimeSteps] containing Vec3 with the locations
			of the contact mesh vertices in space.

		@param[in] faces
			A matrix [nFaces x nVerticesPerFace]


	   */
	   void write(const std::string& fileName, const std::string& filePath,
		   const int frame_num) const;

	   /**
	   @param[in] aFaceDataNames
			Cannot include spaces

	   @param[in] aFaceData
			A vector [nDataField] of SimTK::Matrix [nTimeStep x nFaces]
			containing data values for each face in the contact mesh
	   @param[in] aFaceDataTypes
			Options:
			Int8, UInt8, Int16, UInt16, Int32,
			UInt32, Int64, UInt64, Float32, Float64

	   */
	   void appendFaceData(std::string aFaceDataName, SimTK::Vector aFaceData)
	   {
		   _faceDataNames.push_back(aFaceDataName);
		   _faceData.push_back(aFaceData);
	   };

	   void appendPointData(std::string aPointDataNames, SimTK::Vector aPointData)
	   {
		   _pointDataNames.push_back(aPointDataNames);
		   _pointData.push_back(aPointData);

	   };

	   void setDataFormat(SimTK::String format) {
		   _data_format = format.toLower();
	   }

	   void setPointLocations(const SimTK::RowVector_<SimTK::Vec3>& vertices) {
		   _points.resize(vertices.size());
		   for (int i = 0; i < vertices.size(); ++i) {
			   _points[i] = vertices[i];
		   }		   
	   }

	   void setPolygonConnectivity(const SimTK::Matrix& faces) {
		   _polygon_connectivity = faces;
	   }

	   void setPolygonsFromMesh(const SimTK::PolygonalMesh& mesh);

	   void setLineConnectivity(const SimTK::Vector& line) {
		   _line_connectivity = line;
	   }

    protected:
        OutputTables extendRead(const std::string& fileName) const override;

        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    private:
		bool isLittleEndian() const;
		std::string encodeFloatDataVTPBase64(std::vector<float>& data) const;
		std::string encodeIntDataVTPBase64(std::vector<uint32_t>& data) const;

	//Data
	private:
		std::vector<std::string> _faceDataNames;
		std::vector<SimTK::Vector>_faceData;

		std::vector<std::string> _pointDataNames;
		std::vector<SimTK::Vector> _pointData;

		SimTK::Vector_<SimTK::Vec3> _points;
		SimTK::Matrix _polygon_connectivity;
		SimTK::Vector _line_connectivity;

		SimTK::String _data_format;
    };

} // namespace OpenSim

#endif // OPENSIM_VTP_FILE_ADAPTER_H_
