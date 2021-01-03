#include "ObjModel.h"
#include <functional>
#include <iostream>
#include <fstream>
//#define USE_MIPMAP


obj::Material::Material() : tex(0), specular(0, 0, 0, 0), ambient(0, 0, 0, 0) {

}

obj::Material::~Material() {

}

obj::Geometry::Geometry() {
	memset(texcoordBound, 0, sizeof(float) * 4);
}

obj::Geometry::~Geometry() {
}

obj::ObjModel::ObjModel() {
	selectedGeometry = geometries.end();
}

obj::ObjModel::~ObjModel() {

}

bool obj::ObjModel::Load(const std::string &dirpath, const std::string &filename) {
	std::ifstream fin(dirpath + filename);
	if (!fin.is_open())
		return false;
	std::string buf;
	std::string mtlFilename;
	std::string materialName;

	auto geometry = geometries.rbegin();
	while (std::getline(fin, buf).good()) {
		if (buf.size() < 2 || buf[0] == '#')
			continue;
		std::string type;
		std::stringstream ss(buf);
		ss >> type;
		if (0 == type.compare("v")) {
			float x, y, z;
			ss >> x >> y >> z;
			vertices.push_back(space::Point(x, y, z));

		} else if (0 == type.compare("vt")) {
			float x, y;
			ss >> x >> y;
			texcoords.push_back(space::Point2D(x, y));

		} else if (0 == type.compare("g") || 0 == type.compare("o")) {
			std::string name;
			ss >> name;
			Geometry g;
			g.name = name;
			geometries.push_back(g);
			geometry = geometries.rbegin();
		} else if (0 == type.compare("f")) {
			if (geometry == geometries.rend()) {
				Geometry g;
				geometries.push_back(g);
				geometry = geometries.rbegin();
			}

			int v1, v2, v3, vt1, vt2, vt3, vn1, vn2, vn3;
			char dummy;
			if (buf.find("//") != std::string::npos) {
				ss >> v1 >> dummy >> dummy >> vn1
						>> v2 >> dummy >> dummy >> vn2
						>> v3 >> dummy >> dummy >> vn3;
				geometry->vertexIndices.push_back(space::IPoint(v1 - 1, v2 - 1, v3 - 1));
				geometry->normalIndices.push_back(space::IPoint(vn1 - 1, vn2 - 1, vn3 - 1));
			} else {
				int numSlash = 0;
				for (auto const &c : buf) {
					if (c == '/') numSlash++;
				}
				if (numSlash == 0) {
					ss >> v1 >> v2 >> v3;
					geometry->vertexIndices.push_back(space::IPoint(v1 - 1, v2 - 1, v3 - 1));
				} else if (numSlash == 3) {

					ss >> v1 >> dummy >> vt1 >> v2 >> dummy >> vt2 >> v3 >> dummy >> vt3;
					geometry->vertexIndices.push_back(space::IPoint(v1 - 1, v2 - 1, v3 - 1));
					geometry->texcoordIndices.push_back(space::IPoint(vt1 - 1, vt2 - 1, vt3 - 1));
				} else if (numSlash == 6) {
					ss >> v1 >> dummy >> vt1 >> dummy >> vn1
							>> v2 >> dummy >> vt2 >> dummy >> vn2
							>> v3 >> dummy >> vt3 >> dummy >> vn3;
					geometry->vertexIndices.push_back(space::IPoint(v1 - 1, v2 - 1, v3 - 1));
					geometry->texcoordIndices.push_back(space::IPoint(vt1 - 1, vt2 - 1, vt3 - 1));
					geometry->normalIndices.push_back(space::IPoint(vn1 - 1, vn2 - 1, vn3 - 1));
				}
			}
		} else if (0 == type.compare("mtllib")) {
			ss >> mtlFilename;
			LoadMaterial(dirpath, mtlFilename);
		} else if (0 == type.compare("usemtl")) {
			ss >> materialName;
			geometry->materialName = materialName;
		}
	}
	fin.close();
	//ComputeTexcoordBound();
	//RemoveRedundantVertices();
	ComputeNormalVectors();
	ComputeBoundingBox();
	if (geometries.size() > 0)
		selectedGeometry = geometries.begin();
	for (auto &texCoord : texcoords) {
		int x = (texCoord.x * 1024.0f - 0.5f);
		int y = (texCoord.y * 1024.0f - 0.5f);
		texCoord = space::Point2D((x + 0.5f) / 1024.0f, (y + 0.5f) / 1024.0f);
	}
	return true;
}

bool obj::ObjModel::Save(const std::string &dirpath, const std::string &filename) {
	std::ofstream fout(dirpath + filename);
	if (!fout.is_open())
		return false;
	bool materialExist = false;
	if (!materials.empty()) {
		std::string mtlFilename(filename + ".mtl");
		if (SaveMaterial(dirpath, mtlFilename)) {
			fout << "mtllib " << mtlFilename << std::endl << std::endl;
			materialExist = true;
		}
	}
	for (auto const &vertex : vertices) {
		fout << "v " << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
	}
	for (auto const &texcoord : texcoords) {
		fout << "vt " << texcoord.x << " " << texcoord.y << " " << std::endl;
	}
	for (auto const &normal : normals) {
		fout << "vn " << normal.x << " " << normal.y << " " << normal.z << std::endl;
	}
	fout << std::endl;
	for (auto const &geometry : geometries) {
		fout << "g " << geometry.name << std::endl;
		if (materialExist && (!geometry.materialName.empty())) {
			fout << "usemtl " << geometry.materialName << std::endl;
		}
		auto const &vertexIndices = geometry.vertexIndices;
		auto const &texcoordIndices = geometry.texcoordIndices;

		if (texcoordIndices.empty()) {
			if (normals.empty()) {
				//vertex only
				for (auto const &index : vertexIndices) {
					fout << "f " << index.x + 1 << " " << index.y + 1 << " "
							<< index.z + 1 << std::endl;
				}
			} else {
				//vertex and normal
				for (auto const &index : vertexIndices) {
					fout << "f " << index.x + 1 << "//" << index.x + 1 << " "
							<< index.y + 1 << "//" << index.y + 1 << " "
							<< index.z + 1 << "//" << index.z + 1 << std::endl;
				}
			}
		} else {
			if (normals.empty()) {
				//vertex and texcoord
				for (int i = 0; i < vertexIndices.size(); i++) {
					auto const &vertexIndex = vertexIndices[i];
					auto const &texcoordIndex = texcoordIndices[i];
					fout << "f " << vertexIndex.x + 1 << "/" << texcoordIndex.x + 1 << " "
							<< vertexIndex.y + 1 << "/" << texcoordIndex.y + 1 << " "
							<< vertexIndex.z + 1 << "/" << texcoordIndex.z + 1 << std::endl;
				}
			} else {
				//everything
				for (int i = 0; i < vertexIndices.size(); i++) {
					auto const &vertexIndex = vertexIndices[i];
					auto const &texcoordIndex = texcoordIndices[i];
					fout << "f " << vertexIndex.x + 1 << "/" << texcoordIndex.x + 1 << "/" << vertexIndex.x + 1 << " "
							<< vertexIndex.y + 1 << "/" << texcoordIndex.y + 1 << "/" << vertexIndex.y + 1 << " "
							<< vertexIndex.z + 1 << "/" << texcoordIndex.z + 1 << "/" << vertexIndex.z + 1 << std::endl;
				}
			}
		}
		fout << std::endl;
	}
	fout.close();
	return true;
}

bool obj::ObjModel::LoadMaterial(const std::string &dirpath, const std::string &filename) {
	std::ifstream fin(dirpath + filename);
	if (!fin.is_open())
		return false;
	std::string buf;
	std::string materialName;
	auto material = materials.end();
	while (std::getline(fin, buf).good()) {
		if (buf.size() < 2 || buf[0] == '#')
			continue;
		std::string type;
		std::stringstream ss(buf);
		ss >> type;
		if (0 == type.compare("newmtl")) {
			std::string name;
			ss >> name;
			auto insertRet = materials.insert(std::make_pair(name, Material()));
			material = insertRet.first;
		}
		if (0 == type.compare("Ka")) {
			float r, g, b;
			ss >> r >> g >> b;
			material->second.ambient = RGBA(r, g, b, 1.0f);
		} else if (0 == type.compare("Kd")) {
			float r, g, b;
			ss >> r >> g >> b;
			material->second.diffuse = RGBA(r, g, b, 1.0f);
		} else if (0 == type.compare("Ks")) {
			float r, g, b;
			ss >> r >> g >> b;
			material->second.specular = RGBA(r, g, b, 1.0f);
		} else if (0 == type.compare("d")) {
			float a;
			ss >> a;
			material->second.ambient.a = a;
			material->second.diffuse.a = a;
			material->second.specular.a = a;
		} else if (0 == type.compare("Tr")) {
			float a;
			ss >> a;
			a = 1.0f - a;
			material->second.ambient.a = a;
			material->second.diffuse.a = a;
			material->second.specular.a = a;
		} else if (0 == type.compare("map_Kd")) {
			std::string textureName;
			ss >> textureName;
			cv::Mat texImg = cv::imread(dirpath + textureName);
			if (!texImg.empty()) {
				material->second.texName = textureName;
				cv::flip(texImg, texImg, 0);
				glGenTextures(2, &material->second.tex);
				for (int i = 0; i < 2; i++) {
					glBindTexture(GL_TEXTURE_2D, material->second.tex);
#ifdef USE_MIPMAP
					glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
					glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
#else
					glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
#endif
					glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
#ifndef ANDROID
					glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImg.cols, texImg.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, texImg.data);
#else
					cv::cvtColor(texImg, texImg, CV_BGR2RGB);
					glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImg.cols, texImg.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, texImg.data);
#endif
				}
				texImg.copyTo(material->second.texImage);
				glBindTexture(GL_TEXTURE_2D, 0);
			}
		}
	}
	fin.close();
	return true;
}

bool obj::ObjModel::SaveMaterial(const std::string &dirpath, const std::string &filename) {
	std::ofstream fout(dirpath + filename);
	if (!fout.is_open())
		return false;
	for (auto const &materialPair : materials) {
		fout << "newmtl " << materialPair.first << std::endl;
		const Material &material = materialPair.second;
		fout << "Ka " << material.ambient.r << " " << material.ambient.g << " "
				<< material.ambient.b << " " << material.ambient.a << std::endl;
		fout << "Kd " << material.diffuse.r << " " << material.diffuse.g << " "
				<< material.diffuse.b << " " << material.diffuse.a << std::endl;
		fout << "Ks " << material.specular.r << " " << material.specular.g << " "
				<< material.specular.b << " " << material.specular.a << std::endl;
		fout << "map_Kd " << material.texName << std::endl;
	}
	fout.close();
	return true;
}

void obj::ObjModel::Draw() {
	//for (auto const &geometry : geometries)
	auto const &geometry = *selectedGeometry;
	{
		auto materialKeyVal = materials.find(geometry.materialName);
		if (materialKeyVal != materials.end()) {
			auto material = materialKeyVal->second;
			if (glIsEnabled(GL_LIGHTING)) {
				glMaterialfv(GL_FRONT, GL_AMBIENT, (float *) &(material.ambient));
				glMaterialfv(GL_FRONT, GL_DIFFUSE, (float *) &(material.diffuse));
				glMaterialfv(GL_FRONT, GL_SPECULAR, (float *) &(material.specular));
			} else {
				glColor4f(material.diffuse.r, material.diffuse.g, material.diffuse.b, material.diffuse.a);
			}
			if (material.tex) {
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, material.tex);
			}
		}
		if (geometry.texcoordIndices.size() == 0) {
#ifndef ANDROID
			glBegin(GL_TRIANGLES);
			for (int i = 0; i < geometry.vertexIndices.size(); i++) {
				const auto vIdx = geometry.vertexIndices[i];
				const space::Point &v1 = vertices[vIdx.x];
				const space::Point &v2 = vertices[vIdx.y];
				const space::Point &v3 = vertices[vIdx.z];
				const space::Vector &n1 = normals[vIdx.x];
				const space::Vector &n2 = normals[vIdx.y];
				const space::Vector &n3 = normals[vIdx.z];
				glNormal3f(n1.x, n1.y, n1.z);
				glVertex3f(v1.x, v1.y, v1.z);
				glNormal3f(n2.x, n2.y, n2.z);
				glVertex3f(v2.x, v2.y, v2.z);
				glNormal3f(n3.x, n3.y, n3.z);
				glVertex3f(v3.x, v3.y, v3.z);
			}
			glEnd();
#else
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_NORMAL_ARRAY);
			glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
			glNormalPointer(GL_FLOAT, 0, &normals[0]);
			glDrawElements(GL_TRIANGLES, geometry.vertexIndices.size() * 3, GL_UNSIGNED_INT, &vertices[0]);
			for (int i = 0; i < geometry.vertexIndices.size(); i++)
			{
				const auto vIdx = geometry.vertexIndices[i];
				const Point &v1 = vertices[vIdx.x()];
				const Point &v2 = vertices[vIdx.y()];
				const Point &v3 = vertices[vIdx.z()];
				const Vector &n1 = normals[vIdx.x()];
				const Vector &n2 = normals[vIdx.y()];
				const Vector &n3 = normals[vIdx.z()];
				glNormal3f(n1.x(), n1.y(), n1.z());
				glVertex3f(v1.x(), v1.y(), v1.z());
				glNormal3f(n2.x(), n2.y(), n2.z());
				glVertex3f(v2.x(), v2.y(), v2.z());
				glNormal3f(n3.x(), n3.y(), n3.z());
				glVertex3f(v3.x(), v3.y(), v3.z());
			}
			glEnd();
#endif
		} else {
			glBegin(GL_TRIANGLES);
			for (int i = 0; i < geometry.vertexIndices.size(); i++) {
				const auto vIdx = geometry.vertexIndices[i];
				const auto tIdx = geometry.texcoordIndices[i];
				const space::Point2D &t1 = texcoords[tIdx.x];
				const space::Point2D &t2 = texcoords[tIdx.y];
				const space::Point2D &t3 = texcoords[tIdx.z];
				const space::Point &v1 = vertices[vIdx.x];
				const space::Point &v2 = vertices[vIdx.y];
				const space::Point &v3 = vertices[vIdx.z];
				const space::Vector &n1 = normals[vIdx.x];
				const space::Vector &n2 = normals[vIdx.y];
				const space::Vector &n3 = normals[vIdx.z];
				glNormal3f(n1.x, n1.y, n1.z);
				glTexCoord2f(t1.x, t1.y);
				glVertex3f(v1.x, v1.y, v1.z);
				glNormal3f(n2.x, n2.y, n2.z);
				glTexCoord2f(t2.x, t2.y);
				glVertex3f(v2.x, v2.y, v2.z);
				glNormal3f(n3.x, n3.y, n3.z);
				glTexCoord2f(t3.x, t3.y);
				glVertex3f(v3.x, v3.y, v3.z);
			}
			glEnd();
		}
		glDisable(GL_TEXTURE_2D);
	}
}

void obj::ObjModel::DrawSilhouette(float r, float g, float b) {
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	auto const &geometry = *selectedGeometry;
	{
		glColor3f(r, g, b);
		glBegin(GL_TRIANGLES);
		for (int i = 0; i < geometry.vertexIndices.size(); i++) {
			const auto vIdx = geometry.vertexIndices[i];
			const space::Point &v1 = vertices[vIdx.x];
			const space::Point &v2 = vertices[vIdx.y];
			const space::Point &v3 = vertices[vIdx.z];
			glVertex3f(v1.x, v1.y, v1.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v3.x, v3.y, v3.z);
		}
		glEnd();
	}
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_TEXTURE_2D);
}

void obj::ObjModel::DrawWireframe(float r, float g, float b) {
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	auto const &geometry = *selectedGeometry;
	{
		glColor3f(r, g, b);
		glBegin(GL_TRIANGLES);
		for (int i = 0; i < geometry.vertexIndices.size(); i++) {
			const auto vIdx = geometry.vertexIndices[i];
			const space::Point &v1 = vertices[vIdx.x];
			const space::Point &v2 = vertices[vIdx.y];
			const space::Point &v3 = vertices[vIdx.z];
			glVertex3f(v1.x, v1.y, v1.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v3.x, v3.y, v3.z);
		}
		glEnd();
	}
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_TEXTURE_2D);
}

void obj::ObjModel::DrawTextureMap() {
	auto const &geometry = *selectedGeometry;
	//for (auto const &geometry : geometries)
	{
		auto materialKeyVal = materials.find(geometry.materialName);
		if (materialKeyVal != materials.end()) {
			auto material = materialKeyVal->second;
			if (glIsEnabled(GL_LIGHTING)) {
				glMaterialfv(GL_FRONT, GL_AMBIENT, (float *) &(material.ambient));
				glMaterialfv(GL_FRONT, GL_DIFFUSE, (float *) &(material.diffuse));
				glMaterialfv(GL_FRONT, GL_SPECULAR, (float *) &(material.specular));
			} else {
				glColor4fv((float *) &(material.diffuse));
			}
			if (material.tex) {
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, material.tex);
			}
		}
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		//glOrtho(geometry.texcoordBound[0], geometry.texcoordBound[0] + geometry.texcoordBound[2],
		//	geometry.texcoordBound[1], geometry.texcoordBound[1] + geometry.texcoordBound[3],
		//	-1.0f, 1.0f);
		glOrtho(0, 1, 0, 1, -1.0f, 1.0f);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glBegin(GL_TRIANGLES);
		for (int i = 0; i < geometry.vertexIndices.size(); i++) {
			auto tIdx = geometry.texcoordIndices[i];
			space::Point2D &t1 = texcoords[tIdx.x];
			space::Point2D &t2 = texcoords[tIdx.y];
			space::Point2D &t3 = texcoords[tIdx.z];
			glTexCoord2f(t1.x, t1.y);
			glVertex2f(t1.x, t1.y);
			glTexCoord2f(t2.x, t2.y);
			glVertex2f(t2.x, t2.y);
			glTexCoord2f(t3.x, t3.y);
			glVertex2f(t3.x, t3.y);
		}
		glEnd();
		glDisable(GL_TEXTURE_2D);

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}
}

void obj::ObjModel::ComputeTexcoordBound() {
	for (auto &geometry : geometries) {
		float xmin = geometry.texcoordBound[0];
		float ymin = geometry.texcoordBound[1];
		float xmax = geometry.texcoordBound[0] + geometry.texcoordBound[2];
		float ymax = geometry.texcoordBound[1] + geometry.texcoordBound[3];
		for (int i = 0; i < geometry.texcoordIndices.size(); i++) {
			auto tIdx = geometry.texcoordIndices[i];
			space::Point2D tc[3];
			tc[0] = texcoords[tIdx.x];
			tc[1] = texcoords[tIdx.y];
			tc[2] = texcoords[tIdx.z];
			for (int i = 0; i < 3; i++) {
				xmin = std::min(xmin, (float) tc[0].x);
				xmax = std::max(xmax, (float) tc[0].x);
				ymin = std::min(ymin, (float) tc[0].y);
				ymax = std::max(ymax, (float) tc[0].y);
			}
		}
		geometry.texcoordBound[0] = xmin;
		geometry.texcoordBound[1] = ymin;
		geometry.texcoordBound[2] = xmax - xmin;
		geometry.texcoordBound[3] = ymax - ymin;
	}
}

class CompareRedundantIndexMap : std::binary_function<std::pair<int, int>, std::pair<int, int>, bool> {
public:
	bool operator()(const std::pair<int, int> &p1, const std::pair<int, int> &p2) {
		return (p1.first < p2.first);
	}
};

//void ObjModel::RemoveRedundantVertices()
//{
//	std::cout << "# of original vertices: " << vertices.size() << std::endl;
//	std::vector<std::pair<int, int> > redundantIndexMap;
//	for (int i = 0; i < vertices.size() - 1; i++)
//	{
//		for (int j = i + 1; j < vertices.size(); j++)
//		{
//			if (vertices[i] == vertices[j])
//			{
//				redundantIndexMap.push_back(std::make_pair(j, i));
//			}
//		}
//	}
//	std::sort(redundantIndexMap.begin(), redundantIndexMap.end(), CompareRedundantIndexMap());
//	std::cout << redundantIndexMap.size() << " redundant vertices found:" << std::endl;
//	for (auto const &indexMap : redundantIndexMap)
//	{
//		std::cout << indexMap.first << "-" << indexMap.second << " ";
//	}
//	std::cout << std::endl;
//	for (int k = redundantIndexMap.size() - 1; k >= 0; k--)
//	{
//		vertices.erase(vertices.begin() + redundantIndexMap[k].first);
//	}
//	for (auto &geometry : geometries)
//	{
//		for (int i = 0; i < geometry.vertexIndices.size(); i++)
//		{
//			auto const &vIdx = geometry.vertexIndices[i];
//			int idx[3] = { vIdx.x(), vIdx.y(), vIdx.z() };
//			for (int j = 0; j < 3; j++)
//			{
//				for (int k = redundantIndexMap.size() - 1; k >= 0; k--)
//				{
//					if (idx[j] == redundantIndexMap[k].first)
//					{
//						idx[j] = redundantIndexMap[k].second;
//						break;
//					}
//				}
//			}
//			geometry.vertexIndices[i] = IPoint(idx[0], idx[1], idx[2]);
//		}
//	}
//	for (auto &geometry : geometries)
//	{
//		for (int i = 0; i < geometry.vertexIndices.size(); i++)
//		{
//			auto const &vIdx = geometry.vertexIndices[i];
//			int idx[3] = { vIdx.x(), vIdx.y(), vIdx.z() };
//			for (int j = 0; j < 3; j++)
//			{
//				for (int k = redundantIndexMap.size() - 1; k >= 0 ; k--)
//				{
//					if (idx[j] > redundantIndexMap[k].first)
//					{
//						idx[j] -= (k + 1);
//						break;
//					}
//				}
//			}
//			geometry.vertexIndices[i] = IPoint(idx[0], idx[1], idx[2]);
//		}
//	}
//	for (auto &geometry : geometries)
//	{
//		for (int i = 0; i < geometry.vertexIndices.size(); i++)
//		{
//			auto const &vIdx = geometry.vertexIndices[i];
//			auto const &tIdx = geometry.texcoordIndices[i];
//			mapVertexToTexcoord.insert(std::make_pair(vIdx.x(), tIdx.x()));
//			mapVertexToTexcoord.insert(std::make_pair(vIdx.y(), tIdx.y()));
//			mapVertexToTexcoord.insert(std::make_pair(vIdx.z(), tIdx.z()));
//		}
//	}	
//	std::cout << "# of reduced vertices: " << vertices.size() << std::endl;
//	Save("reduces.obj");
//}

void obj::ObjModel::ComputeNormalVectors() {
	normals.resize(vertices.size(), space::Vector(0, 0, 0));
	for (auto const &geometry : geometries) {
		for (auto const &index : geometry.vertexIndices) {
			int idx[3] = {index.x, index.y, index.z};
			const space::Point &v1 = vertices[idx[0]];
			const space::Point &v2 = vertices[idx[1]];
			const space::Point &v3 = vertices[idx[2]];
			space::Vector n = space::Vector::GetNormalFromPts(v1, v2, v3); //Vector(v1, v2, v3); // normal for v2 - v1, v3 - v1			
			normals[idx[0]] += n;
			normals[idx[1]] += n;
			normals[idx[2]] += n;
		}
	}
	for (auto &normal : normals) {
		float norm = sqrtf(normal.squared_length());
		if (norm == 0)
			normal = space::Vector(0, 0, 0);
		else
			normal /= norm;
	}
}

void obj::ObjModel::ComputeBoundingBox() {
	float minX = 99999999999;
	float minY = 99999999999;
	float minZ = 99999999999;
	float maxX = -99999999999;
	float maxY = -99999999999;
	float maxZ = -99999999999;

	for (auto const &geometry : geometries) {
		for (auto const &index : geometry.vertexIndices) {
			int idx[3] = {index.x, index.y, index.z};
			for (int i = 0; i < 3; i++) {
				const space::Point &v = vertices[idx[i]];
				minX = std::min(minX, v.x);
				minY = std::min(minY, v.y);
				minZ = std::min(minZ, v.z);
				maxX = std::max(maxX, v.x);
				maxY = std::max(maxY, v.y);
				maxZ = std::max(maxZ, v.z);
			}
		}
	}
	boundingBox = std::make_pair(space::Point(maxX, maxY, maxZ), space::Point(minX, minY, minZ));
}

int obj::ObjModel::GetNumGeometries() {
	return geometries.size();
}

std::vector<obj::Geometry> obj::ObjModel::GetGeometries() {
	return geometries;
}

std::vector<space::Point> obj::ObjModel::GetVertices() {
	return vertices;
}

std::vector<space::Point2D> obj::ObjModel::GetTexcoord() {
	return texcoords;
}

std::vector<space::IPoint> obj::ObjModel::GetTexcoordIndices() {
	return selectedGeometry->texcoordIndices;
}

std::vector<space::IPoint> obj::ObjModel::GetVertexIndices() {
	return selectedGeometry->vertexIndices;
}

void obj::ObjModel::DeformUsingFeatures(std::vector<FeaturePoint> &features) {
	space::Vector dia = boundingBox.first - boundingBox.second;
	float scale = sqrt(dia.squared_length());
	float nnFactor = 0.2f;
	std::map<int, std::list<int> > verticesAssociatedFeatures;
	std::vector<space::Vector> vertexMotionVectors(vertices.size());
	for (int i = 0; i < features.size(); i++) {
		FeaturePoint &feature = features[i];
		feature.motionVector = feature.currentPt3D - feature.prevPt3D;
		int vertexIdx[3] = {feature.vertexIdx.x, feature.vertexIdx.y, feature.vertexIdx.z};
		for (int j = 0; j < 3; j++) {
			verticesAssociatedFeatures[vertexIdx[j]].push_back(i);
		}
	}
	for (int vIdx = 0; vIdx < vertices.size(); vIdx++) {
		auto vit = verticesAssociatedFeatures.find(vIdx);
		//	vertex�� �����ϴ� face ���� feature�� ���� ���
		if (vit != verticesAssociatedFeatures.end()) {
			std::vector<space::Vector> featureMotionVectors;
			std::vector<float> distances;
			std::vector<float> weights;
			space::Vector motionVector;
			float distSum = 0.0f;
			for (auto &fid : vit->second) {
				featureMotionVectors.push_back(features[fid].motionVector);
				auto dvec = features[fid].currentPt3D - vertices[vIdx];
				distances.push_back(sqrt(dvec.squared_length()));
				distSum += distances.back();
			}
			if (distances.size() == 1) {
				weights.push_back(1);
			} else {
				if (distSum > 0) {
					for (auto dist : distances) {
						weights.push_back(1.0f - (dist / distSum));
						//std::cout << "dist: " << dist << "\t distsum: " << distSum << std::endl;
					}
					float wsum = 0;
					for (auto w : weights) {
						wsum += w;
					}
					for (auto &w : weights) {
						w = w / wsum;
					}
				} else {
					float w = 1.0f / featureMotionVectors.size();
					weights.resize(featureMotionVectors.size(), w);
				}
			}

			for (int n = 0; n < featureMotionVectors.size(); n++) {
				motionVector += (featureMotionVectors[n] * weights[n]);
			}
			vertexMotionVectors[vIdx] = motionVector;
		} else {
			std::vector<int> nnfIDs; // IDs of nearest neighbor features
			for (int fIdx = 0; fIdx < features.size(); fIdx++) {
				space::Vector diff = features[fIdx].currentPt3D - vertices[vIdx];
				float dist = sqrt(diff.squared_length());
				//std::cout << "dist: " << dist << "\t scale: " << scale << "\t nnfactor: " << nnFactor << std::endl;
				if (dist < scale * nnFactor) {
					nnfIDs.push_back(fIdx);
				}
			}
			if (nnfIDs.size() == 0)
				continue;
			std::vector<space::Vector> featureMotionVectors;
			std::vector<float> distances;
			std::vector<float> weights;
			space::Vector motionVector;
			float distSum = 0.0f;
			for (auto &fid : nnfIDs) {
				featureMotionVectors.push_back(features[fid].motionVector);
				auto dvec = features[fid].currentPt3D - vertices[vIdx];
				distances.push_back(sqrt(dvec.squared_length()));
				distSum += distances.back();
			}
			if (distances.size() == 1) {
				weights.push_back(1);
			} else {
				if (distSum > 0) {
					for (auto dist : distances) {
						weights.push_back(1.0f - (dist / distSum));
					}
					float wsum = 0;
					for (auto w : weights) {
						wsum += w;
					}
					for (auto &w : weights) {
						w = w / wsum;
					}
				} else {
					float w = 1.0f / distances.size();
					weights.resize(distances.size(), w);
				}
			}

			for (int n = 0; n < featureMotionVectors.size(); n++) {
				motionVector += (featureMotionVectors[n] * weights[n]);
			}
			vertexMotionVectors[vIdx] = motionVector;
		}
	}
	static int frameCnt = 0;
	if (frameCnt < 30) {
		char filename[1000];
		sprintf(filename, "motion_vertex_%06d.txt", frameCnt);
		FILE *fp = fopen(filename, "w");
		for (int vIdx = 0; vIdx < vertices.size(); vIdx++) {
			vertices[vIdx] += vertexMotionVectors[vIdx];
			fprintf(fp, "%f\t%f\t%f\n", vertexMotionVectors[vIdx].x, vertexMotionVectors[vIdx].y, vertexMotionVectors[vIdx].z);
		}
		fclose(fp);
		{
			sprintf(filename, "motion_feature_%06d.txt", frameCnt);
			FILE *fp = fopen(filename, "w");
			for (auto feature : features) {
				fprintf(fp, "%f\t%f\t%f\n", feature.motionVector.x, feature.motionVector.y, feature.motionVector.z);
			}
			fclose(fp);
		}
	}
	frameCnt++;
}
