#include "Model.hpp"
#include <cassert>

namespace Impact {
namespace Rendering3D {

Model::Model(const TriangleMesh* new_mesh,
			 const Material* new_material,
			 const AffineTransformation& new_transformation)
	: _mesh(new_mesh),
	  _material(new_material),
	  _transformation(new_transformation) {}

Model::Model(const TriangleMesh* new_mesh,
			 const Material* new_material)
	: _mesh(new_mesh),
	  _material(new_material) {}

void Model::setTexture(const Texture* texture)
{
	assert(_mesh->hasTextureCoordinates());
	_texture = texture;
}

void Model::setBumpMap(const Texture* bump_map)
{
	assert(_mesh->hasTextureCoordinates());
	_bump_map = bump_map;
}

void Model::setDisplacementMap(const Texture* displacement_map)
{
	assert(_mesh->hasTextureCoordinates());
	_displacement_map = displacement_map;
}

void Model::applyTransformation(const AffineTransformation& transformation)
{
	_transformation = transformation(_transformation);
}

Geometry3D::TriangleMesh Model::getTransformedMesh() const
{
	return TriangleMesh(*_mesh).applyTransformation(_transformation);
}

Geometry3D::TriangleMesh Model::getTransformedMesh(const AffineTransformation& additional_transformation) const
{
	return TriangleMesh(*_mesh).applyTransformation(additional_transformation(_transformation));
}

const Geometry3D::TriangleMesh* Model::getMesh() const
{
	return _mesh;
}

const Material* Model::getMaterial() const
{
	return _material;
}

const Geometry3D::AffineTransformation& Model::getTransformation() const
{
	return _transformation;
}

bool Model::hasTexture() const
{
	return _texture;
}

bool Model::hasBumpMap() const
{
	return _bump_map;
}

bool Model::hasDisplacementMap() const
{
	return _displacement_map;
}

const Texture* Model::getTexture() const
{
	return _texture;
}

const Texture* Model::getBumpMap() const
{
	return _bump_map;
}

const Texture* Model::getDisplacementMap() const
{
	return _displacement_map;
}

/*RenderableTriangleMesh RenderableTriangleMesh::file(const std::string& filename)
{
	// Materials

    assert(n_material_names <= 1);

    imp_uint n_materials;
    imp_uint n;
	imp_uint idx;

    for (n = 0; n < n_material_files; n++)
    {
        const std::vector<BlinnPhongMaterial>& materials = _getMtlFileData(material_files[n]);
        n_materials = static_cast<imp_uint>(materials.size());
    
        for (idx = 0; idx < n_materials; idx++)
        {
            if (materials[idx].getName() == material_names[0])
            {
                mesh.setMaterial(materials[idx]);
                break;
            }
        }

        if (mesh._has_material) break;
    }

    return mesh;
}

std::vector<BlinnPhongMaterial> RenderableTriangleMesh::_getMtlFileData(const std::string& filename)
{
    imp_uint length = static_cast<imp_uint>(filename.length());
    assert(length > 4 &&
           filename.substr(length-4, 4) == ".mtl");

    std::ifstream infile(filename);
    assert(infile);

    std::string line, word, current_material = "";
    imp_uint split;
    bool has_material = false;

    std::map<std::string, std::map<std::string, std::string> > material_properties;

    while (std::getline(infile, line))
    {
        string_util::trim(line);

        split = static_cast<imp_uint>(line.find(" "));

        if (split == static_cast<imp_uint>(std::string::npos))
            continue;

        word = line.substr(0, split);

        if (word == "#")
        {
            continue;
        }
        else if (word == "newmtl")
        {
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);
            
            if (material_properties.find(line) == material_properties.end())
                material_properties.insert(std::make_pair(line, std::map<std::string, std::string>()));

            current_material = line;
            has_material = true;
        }
        else
        {
            assert(has_material);

            std::map<std::string, std::string>& properties = material_properties[current_material];
            
            line = line.substr(split + 1, line.length() - split - 1);
            string_util::ltrim(line);
            assert(line.length() > 0);

            if (properties.find(line) == properties.end())
                properties.insert(std::make_pair(word, line));
            else
                properties[word] = line;
        }
    }

    std::vector<BlinnPhongMaterial> materials;

    string_vec splitted;
    imp_uint n_values;

    Reflectance diffuse_reflectance;
    Reflectance glossy_reflectance;
    float smoothness;

    std::map<std::string, std::map<std::string, std::string> >::const_iterator iter;
    std::map<std::string, std::string>::const_iterator property_iter;

    for (iter = material_properties.begin(); iter != material_properties.end(); iter++)
    {
        const std::map<std::string, std::string>& properties = iter->second;

        
        diffuse_reflectance = Reflectance::black();
        glossy_reflectance = Reflectance::black();
        smoothness = 0.0f;

        for (property_iter = properties.begin(); property_iter != properties.end(); property_iter++)
        {
            if (property_iter->first == "Kd")
            {
                splitted = string_util::split(property_iter->second);
                n_values = static_cast<imp_uint>(splitted.size());
                assert(n_values == 3);

                diffuse_reflectance.r = static_cast<imp_float>(atof(splitted[0].c_str()));
                diffuse_reflectance.g = static_cast<imp_float>(atof(splitted[1].c_str()));
                diffuse_reflectance.b = static_cast<imp_float>(atof(splitted[2].c_str()));
            }
            else if (property_iter->first == "Ks")
            {
                splitted = string_util::split(property_iter->second);
                n_values = static_cast<imp_uint>(splitted.size());
                assert(n_values == 3);

                glossy_reflectance.r = static_cast<imp_float>(atof(splitted[0].c_str()));
                glossy_reflectance.g = static_cast<imp_float>(atof(splitted[1].c_str()));
                glossy_reflectance.b = static_cast<imp_float>(atof(splitted[2].c_str()));
            }
            else if (property_iter->first == "Ns")
            {
                splitted = string_util::split(property_iter->second);
                n_values = static_cast<imp_uint>(splitted.size());
                assert(n_values == 1);

                smoothness = static_cast<imp_float>(atof(splitted[0].c_str()));
            }
        }

        materials.push_back(BlinnPhongMaterial(diffuse_reflectance,
                                               glossy_reflectance,
                                               smoothness,
                                               iter->first));
    }

    return materials;
}*/

//void SceneObject::saveAs(const std::string& filename, const std::string& material_filename /* = std::string() */) const
/*{
    imp_uint length = static_cast<imp_uint>(filename.length());
    assert(length > 4 &&
           filename.substr(length-4, 4) == ".obj");

    std::ofstream outfile(filename);
    assert(outfile);

    std::string material_name = "default";

    bool include_material = _has_material && !material_filename.empty();

    if (include_material)
    {
        length = static_cast<imp_uint>(material_filename.length());
        assert(length > 4 &&
               material_filename.substr(length-4, 4) == ".mtl");

        material_name = _material->getName();

        outfile << "mtllib " << material_filename << std::endl << std::endl;
    }

    imp_uint n_vertices = getNumberOfVertices();
    imp_uint n_faces = getNumberOfFaces();
    imp_uint idx;

    for (idx = 0; idx < n_vertices; idx++)
    {
        outfile << "v " << _vertices(0, idx) << " " << 
                           _vertices(1, idx) << " " <<
                           _vertices(2, idx) << " " <<
                           _vertices(3, idx) <<
                           std::endl;
    }

    outfile << std::endl;

    if (_has_normals)
    {
        for (idx = 0; idx < n_vertices; idx++)
        {
            outfile << "vn " << _normals(0, idx) << " " << 
                                _normals(1, idx) << " " <<
                                _normals(2, idx) <<
                                std::endl;
        }
        
        outfile << std::endl;

        if (include_material)
        {
            outfile << "g Object" << std::endl;
            outfile << "usemtl " << material_name << std::endl;
        }

        for (idx = 0; idx < n_faces; idx++)
        {
            outfile << "f " << _faces(0, idx)+1 << "//" << _faces(0, idx)+1 << " " <<
                               _faces(1, idx)+1 << "//" << _faces(1, idx)+1 << " " <<
                               _faces(2, idx)+1 << "//" << _faces(2, idx)+1 << std::endl;
        }
    }
    else
    {
        if (include_material)
        {
            outfile << "g Object" << std::endl;
            outfile << "usemtl " << material_name << std::endl;
        }

        for (idx = 0; idx < n_faces; idx++)
        {
            outfile << "f " << _faces(0, idx)+1 << " " << 
                               _faces(1, idx)+1 << " " <<
                               _faces(2, idx)+1 << std::endl;
        }
    }

    outfile.close();
}*/

} // Rendering3D
} // Impact
