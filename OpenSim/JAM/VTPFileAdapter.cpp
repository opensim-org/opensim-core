/* -------------------------------------------------------------------------- *
 *                            VTPFileAdapter.cpp                              *
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
#include "VTPFileAdapter.h"
#include <fstream>
#include "base64.h"
#include <bitset>

using namespace SimTK;
namespace OpenSim {


    VTPFileAdapter* VTPFileAdapter::clone() const
    {
        return new VTPFileAdapter{ *this };
    }


    VTPFileAdapter::OutputTables VTPFileAdapter::extendRead(const std::string& fileName) const
    {
        OPENSIM_THROW(Exception,"VTPFileAdapter::extendRead is not implemented.")
        OutputTables output_tables{};
        return output_tables;
    };

    void VTPFileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
    {
    };

    void VTPFileAdapter::setPolygonsFromMesh(const SimTK::PolygonalMesh& mesh) {
        
        //Find Vertex locations
        _points.resize(mesh.getNumVertices());

        
        for (int i = 0; i < mesh.getNumVertices(); ++i)
        {
            //Vertex Locations
            SimTK::Vec3 position = mesh.getVertexPosition(i);
            _points(i)(0) = position(0);
            _points(i)(1) = position(1);
            _points(i)(2) = position(2);
        }
        
        //Construct Face connectivity
        _polygon_connectivity.resize(mesh.getNumFaces(), mesh.getNumVerticesForFace(0));

        for (int i = 0; i < mesh.getNumFaces(); ++i) {
            for (int j = 0; j < mesh.getNumVerticesForFace(i); ++j)
            {
                _polygon_connectivity(i, j) = mesh.getFaceVertex(i, j);
            }
        }
    
    }


    //Write Using direct input of points and vertices
    void VTPFileAdapter::write(const std::string& fileName, const std::string& filePath, const int frame_num) const {

        //std::cout << "Writing .vtp files to: " + filePath + fileName << std::endl;

        int nPoints = _points.nrow();
        int nStrips = 0;
        int nVerts = 0;
        int nLines = (_line_connectivity.size()>0)?1:0;
        int nPolys = _polygon_connectivity.nrow();

        Xml::Document doc;

        doc.setRootTag("VTKFile");

        Xml::Element root = doc.getRootElement();
        root.setAttributeValue("type", "PolyData");
        root.setAttributeValue("version", "1.0");
        root.setAttributeValue("header_type", "UInt32");
        if (isLittleEndian()) root.setAttributeValue("byte_order", "LittleEndian");
        else root.setAttributeValue("byte_order", "BigEndian");
        SimTK::Xml::Element PolyData("PolyData");
        root.appendNode(PolyData);

        Xml::Element Piece("Piece");
        Piece.setAttributeValue("NumberOfPoints", std::to_string(nPoints));
        Piece.setAttributeValue("NumberOfVerts", std::to_string(nVerts));
        Piece.setAttributeValue("NumberOfLines", std::to_string(nLines));
        Piece.setAttributeValue("NumberOfStrips", std::to_string(nStrips));
        Piece.setAttributeValue("NumberOfPolys", std::to_string(nPolys));

        PolyData.appendNode(Piece);

        //Point Data
        Xml::Element PointData("PointData");
        std::string pnt_names;
        for (int i = 0; i < (int)_pointDataNames.size(); ++i) {
            pnt_names.append(" " + _pointDataNames[i]);
        }
        PointData.setAttributeValue("Scalars", pnt_names);

        for (int i = 0; i < (int)_pointDataNames.size(); ++i) {
            Xml::Element DataArray("DataArray");
            DataArray.setAttributeValue("type","Float32");
            DataArray.setAttributeValue("Name", _pointDataNames[i]);

            if (_data_format == "ascii") {
                DataArray.setAttributeValue("format", "ascii");

                std::string pdata;
                for (int j = 0; j < _pointData[i].size(); ++j) {
                    pdata.append(std::to_string(_pointData[i](j)) + " ");
                }
                DataArray.setValue(pdata);
            }
                
            else if (_data_format == "binary") {
                DataArray.setAttributeValue("format", "binary");

                std::vector<float> data;
                for (int j = 0; j < _pointData[i].size(); ++j) {
                    data.push_back((float)_pointData[i](j));
                }
                std::string encoded_data = encodeFloatDataVTPBase64(data);
                DataArray.setValue(encoded_data);

            }
            PointData.appendNode(DataArray);
        }
        Piece.appendNode(PointData);


        //Cell Data
        Xml::Element CellData("CellData");
        std::string cell_names;
        for (int i = 0; i < (int)_faceDataNames.size(); ++i) {
            cell_names.append(" " + _faceDataNames[i]);
        }
        CellData.setAttributeValue("Scalars", cell_names);

        for (int i = 0; i < (int)_faceDataNames.size(); ++i) {
            Xml::Element DataArray("DataArray");
            DataArray.setAttributeValue("type", "Float32");
            DataArray.setAttributeValue("Name", _faceDataNames[i]);

            std::string cdata;
            if (_data_format == "ascii") {
                DataArray.setAttributeValue("format", "ascii");
                for (int j = 0; j < _faceData[i].size(); ++j) {
                    cdata.append(std::to_string(_faceData[i](j)) + " ");
                }
                DataArray.setValue(cdata);
            }

            else if (_data_format == "binary") {
                DataArray.setAttributeValue("format", "binary");

                std::vector<float> data;
                for (int j = 0; j < _faceData[i].size(); ++j) {
                    data.push_back((float)_faceData[i](j));
                }

                std::string encode_data = encodeFloatDataVTPBase64(data);
                DataArray.setValue(encode_data);
            }
            CellData.appendNode(DataArray);
        }
        Piece.appendNode(CellData);

        //Points
        Xml::Element Points("Points");
        Xml::Element PointArray("DataArray");
        PointArray.setAttributeValue("Name", "Points");
        PointArray.setAttributeValue("type", "Float32");
        PointArray.setAttributeValue("NumberOfComponents", "3");

            
        if (_data_format == "ascii") {
            PointArray.setAttributeValue("format", "ascii");
            std::string pnt_coords;
            for (int i = 0; i < _points.size(); ++i) {
                pnt_coords.append(std::to_string(_points(i)(0)) + " ");
                pnt_coords.append(std::to_string(_points(i)(1)) + " ");
                pnt_coords.append(std::to_string(_points(i)(2)) + " ");
            }
            PointArray.setValue(pnt_coords);
        }
        else if (_data_format == "binary") {
            PointArray.setAttributeValue("format", "binary");
            std::vector<float> pnt_coords;
            for (int i = 0; i < _points.size(); ++i) {
                pnt_coords.push_back((float)_points(i)(0));
                pnt_coords.push_back((float)_points(i)(1));
                pnt_coords.push_back((float)_points(i)(2));
            }
            std::string encode_data = encodeFloatDataVTPBase64(pnt_coords);
            PointArray.setValue(encode_data);
        }
        Points.appendNode(PointArray);
        Piece.appendNode(Points);

        //Verts
        Xml::Element Verts("Verts");
        Xml::Element VertsData1("DataArray");
        Xml::Element VertsData2("DataArray");
        VertsData1.setAttributeValue("type", "Int64");
        VertsData1.setAttributeValue("Name", "connectivity");
        VertsData2.setAttributeValue("type", "Int64");
        VertsData2.setAttributeValue("Name", "offsets");
        if (_data_format == "binary") {
            VertsData1.setAttributeValue("format", "binary");
            VertsData2.setAttributeValue("format", "binary");
        }
        else if (_data_format == "ascii") {
            VertsData1.setAttributeValue("format", "ascii");
            VertsData2.setAttributeValue("format", "ascii");
        }
        Verts.appendNode(VertsData1);
        Verts.appendNode(VertsData2);
        Piece.appendNode(Verts);

        //Lines
        if (_line_connectivity.size()) {
            Xml::Element Lines("Lines");
            Xml::Element LinesData1("DataArray");
            Xml::Element LinesData2("DataArray");
            LinesData1.setAttributeValue("type", "UInt32");
            LinesData1.setAttributeValue("Name", "connectivity");
            LinesData2.setAttributeValue("type", "UInt32");
            LinesData2.setAttributeValue("Name", "offsets");
            if (_data_format == "ascii") {
                LinesData1.setAttributeValue("format", "ascii");
                LinesData2.setAttributeValue("format", "ascii");

                std::string line_data;
                std::string line_offset;
                int line_off = 0;
                for (int i = 0; i < _line_connectivity.nrow(); ++i) {
                    line_data.append(std::to_string(int(_line_connectivity(i))) + " ");
                    line_off++;
                }
                line_offset.append(std::to_string(line_off) + " ");
                LinesData1.setValue(line_data);
                LinesData2.setValue(line_offset);
            }
            else if (_data_format == "binary") {
                LinesData1.setAttributeValue("format", "binary");
                LinesData2.setAttributeValue("format", "binary");

                std::vector<uint32_t> line_data;
                std::vector<uint32_t> line_offset;
                int line_off = 0;
                for (int i = 0; i < _line_connectivity.size(); ++i) {
                    line_data.push_back(uint32_t(_line_connectivity(i)));
                    line_off++;
                }
                line_offset.push_back(line_off);
                std::string encode_line_data = encodeIntDataVTPBase64(line_data);
                std::string encode_line_offset = encodeIntDataVTPBase64(line_offset);

                LinesData1.setValue(encode_line_data);
                LinesData2.setValue(encode_line_offset);
            }
            Lines.appendNode(LinesData1);
            Lines.appendNode(LinesData2);
            Piece.appendNode(Lines);
        }

        //Strips
        Xml::Element Strips("Strips");
        Xml::Element StripsData1("DataArray");
        Xml::Element StripsData2("DataArray");
        StripsData1.setAttributeValue("type", "Int64");
        StripsData1.setAttributeValue("Name", "connectivity");
        StripsData2.setAttributeValue("type", "Int64");
        StripsData2.setAttributeValue("Name", "offsets");
        if (_data_format == "binary") {
            StripsData1.setAttributeValue("format", "binary");
            StripsData2.setAttributeValue("format", "binary");
        }
        else if (_data_format == "ascii") {
            StripsData1.setAttributeValue("format", "ascii");
            StripsData2.setAttributeValue("format", "ascii");
        }
        Strips.appendNode(StripsData1);
        Strips.appendNode(StripsData2);
        Piece.appendNode(Strips);

        //Polygons
        Xml::Element Polys("Polys");
        Xml::Element PolysData1("DataArray");
        Xml::Element PolysData2("DataArray");
        PolysData1.setAttributeValue("type", "UInt32");
        PolysData1.setAttributeValue("Name", "connectivity");
        PolysData2.setAttributeValue("type", "UInt32");
        PolysData2.setAttributeValue("Name", "offsets");

        if (_data_format == "ascii") {
            PolysData1.setAttributeValue("format", "ascii");
            PolysData2.setAttributeValue("format", "ascii");

            std::string poly_data;
            std::string poly_offset;
            int poly_off=0;
            for (int i = 0; i < _polygon_connectivity.nrow(); ++i) {
                for (int j = 0; j < 3; ++j) {
                    poly_data.append(std::to_string(int(_polygon_connectivity(i, j))) + " ");
                    poly_off++;
                }
                poly_offset.append(std::to_string(poly_off)+" ");
            }
            PolysData1.setValue(poly_data);
            PolysData2.setValue(poly_offset);
        }

        else if (_data_format == "binary") {
            PolysData1.setAttributeValue("format", "binary");
            PolysData2.setAttributeValue("format", "binary");

            std::vector<uint32_t> poly_data;
            std::vector<uint32_t> poly_offset;
            int poly_off = 0;
            for (int i = 0; i < _polygon_connectivity.nrow(); ++i) {
                for (int j = 0; j < 3; ++j) {
                    poly_data.push_back(uint32_t(_polygon_connectivity(i, j)));
                    poly_off++;
                }
                poly_offset.push_back(poly_off);
            }
            std::string encode_poly_data = encodeIntDataVTPBase64(poly_data);
            std::string encode_poly_offset = encodeIntDataVTPBase64(poly_offset);

            PolysData1.setValue(encode_poly_data);
            PolysData2.setValue(encode_poly_offset);

        }

        Polys.appendNode(PolysData1);
        Polys.appendNode(PolysData2);

        Piece.appendNode(Polys);

        //Write File
        doc.writeToFile(filePath + fileName + "_" + std::to_string(frame_num) + ".vtp");
        

    }

    bool VTPFileAdapter::isLittleEndian() const 
    {
        int num = 1;

        if (*(char *)&num == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string VTPFileAdapter::encodeFloatDataVTPBase64(std::vector<float>& data) const {

        size_t nData = data.size();

        const float *p_floats = &(data[0]);
        const unsigned char* p_bytes = reinterpret_cast<const unsigned char *>(p_floats);

        uint32_t len = (uint32_t)(nData * sizeof(float));


        std::string cdata = base64_encode(p_bytes, len);
        std::string encoded_data = base64_encode(reinterpret_cast<unsigned char *>(&len), sizeof(uint32_t));
        encoded_data.append(cdata);
        return encoded_data;
    }

    std::string VTPFileAdapter::encodeIntDataVTPBase64(std::vector<uint32_t>& data) const {

        size_t nData = data.size();

        const uint32_t *p_int = &(data[0]);
        const unsigned char* p_bytes = reinterpret_cast<const unsigned char *>(p_int);
        uint32_t len = (uint32_t)(nData * sizeof(uint32_t));

        std::string cdata = base64_encode(p_bytes, len);
        std::string encoded_data = base64_encode(reinterpret_cast<unsigned char *>(&len), sizeof(uint32_t));
        encoded_data.append(cdata);

        return encoded_data;
    }
}


