#pragma once
#include "precision.hpp"
#include <armadillo>
#include <vector>
#include <string>
#include <map>
#include <utility>

namespace Impact {
namespace Geometry3D {

class HalfEdge;

class MeshTopology {
	
private:
	typedef std::map<std::pair<imp_uint, imp_uint>, HalfEdge> half_edge_map;
	typedef std::vector<HalfEdge*> half_edge_vector;

protected:
	arma::Mat<imp_uint> _faces;

	half_edge_map _half_edges;
	half_edge_vector _outgoing_half_edges;
	half_edge_vector _first_bordering_half_edges;

	bool _has_adjacency_data = false;

	imp_uint _face_idx;

public:

	~MeshTopology();

	void initializeFaces(imp_uint n_faces, bool uses_texture_coordinates);
	
	void reserveAdditionalFaces(imp_uint n_additional_faces);

	void addFace(imp_uint i, imp_uint j, imp_uint k);

	void addFace(imp_uint i, imp_uint j, imp_uint k,
				 imp_uint l, imp_uint m, imp_uint n);

	void getFace(imp_uint face_idx,
				 imp_uint& i, imp_uint& j, imp_uint& k) const;
	
	void getFace(imp_uint face_idx,
				 imp_uint& i, imp_uint& j, imp_uint& k,
				 imp_uint& l, imp_uint& m, imp_uint& n) const;
	
	void getTextureCoordinateIndices(imp_uint face_idx,
									 imp_uint& l, imp_uint& m, imp_uint& n) const;

	void setNextFace(imp_uint i, imp_uint j, imp_uint k);

	void setNextFace(imp_uint i, imp_uint j, imp_uint k,
					 imp_uint l, imp_uint m, imp_uint n);

	void setFace(imp_uint face_idx,
				 imp_uint i, imp_uint j, imp_uint k);

	void setFace(imp_uint face_idx,
				 imp_uint i, imp_uint j, imp_uint k,
				 imp_uint l, imp_uint m, imp_uint n);

	void setFaceComponent(imp_uint component, imp_uint face_idx, imp_uint value);

	void removeFace(imp_uint face_idx);

	void removeFacesContainingVertex(imp_uint vertex_idx);
	void removeFacesContainingVertex(imp_uint vertex_idx, arma::Mat<imp_float>& face_normals);

	void duplicateFaceIndicesForTextureCoordinates();
	void removeTextureCoordinateIndices();

	void generateOppositeFaces();

	void generateAdjacencyData(imp_uint n_vertices);

	void getNeighbourVertices(imp_uint vertex_idx, std::vector<imp_uint>& neighbour_vertices) const;

	void getOppositeFacesAndVertices(imp_uint face_idx,
									 imp_uint opposite_faces[3],
									 imp_uint opposite_vertices[3]) const;

	void invalidateAdjacencyData();

	imp_uint getNumberOfFaces() const;

	bool hasTextureCoordinateIndices() const;
	bool hasAdjacencyData() const;

	std::string getFacesString() const;
};

class HalfEdge {

public:
	HalfEdge* next_half_edge;
	HalfEdge* adjacent_half_edge; // Oppositely oriented to this one

	imp_uint bordered_face_idx;
	imp_uint vertex_at_end_idx;

	HalfEdge();
};

} // Geometry3D
} // Impact
