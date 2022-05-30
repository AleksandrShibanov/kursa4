#include <array>
#include <unordered_set>
#include <vector>
#include <functional>
#include <memory>

#include "hash.hpp"
#include "triangulation3D.hpp"


namespace Triangulation {
constexpr uint32_t STP0_INDEX = std::numeric_limits<uint32_t>::max();
constexpr uint32_t STP1_INDEX = std::numeric_limits<uint32_t>::max() - 1;
constexpr uint32_t STP2_INDEX = std::numeric_limits<uint32_t>::max() - 2;
constexpr uint32_t STP3_INDEX = std::numeric_limits<uint32_t>::max() - 3;
constexpr uint32_t STP4_INDEX = std::numeric_limits<uint32_t>::max() - 4;
constexpr uint32_t STP5_INDEX = std::numeric_limits<uint32_t>::max() - 5;
constexpr uint32_t STP6_INDEX = std::numeric_limits<uint32_t>::max() - 6;
constexpr uint32_t STP7_INDEX = std::numeric_limits<uint32_t>::max() - 7;
constexpr uint32_t STP_START	= STP7_INDEX;

struct Tetrahedron;
using TetrahedronPtr = std::unique_ptr<Tetrahedron>;
struct Face 
{
	uint32_t IV0;
	uint32_t IV1;
	uint32_t IV2;
	Tetrahedron* Neighbor = nullptr;
	bool Bad;

	inline bool hasVertex(uint32_t k) const { return IV0 == k || IV1 == k || IV2 == k; }
	inline bool isValid() const { return IV0 != IV1 && IV2 != IV1 && IV0 != IV2; }

	inline void makeBad()
	{
		Bad = true;
	}

	inline bool isBad() const
	{
		return Bad;
	}
};

inline bool operator==(const Face& a, const Face& b)
{
	return a.hasVertex(b.IV0) && a.hasVertex(b.IV1) && a.hasVertex(b.IV2);
}

struct FaceHash 
{
	std::size_t operator()(Face const& s) const noexcept
	{
		std::size_t h1 = hash_union(std::hash<uint32_t>{}(s.IV0), std::hash<uint32_t>{}(s.IV1));
		hash_combine(h1, std::hash<uint32_t>{}(s.IV2));
		return h1;
	}
};

inline float volumeTetrahedron(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3)
{
	return std::abs((v3 - v0).dot((v2 - v0).cross(v1 - v0))) / 6;
}

inline Eigen::Vector3f circumCenterTetrahedron(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3)
{
	const float nv0 = v0.squaredNorm();
	const float nv1 = v1.squaredNorm();
	const float nv2 = v2.squaredNorm();
	const float nv3 = v3.squaredNorm();

	const Eigen::Vector3f B = 0.5f * Eigen::Vector3f(nv1 - nv0, nv2 - nv0, nv3 - nv0);
	Eigen::Matrix3f A;
	A.row(0) = v1 - v0;
	A.row(1) = v2 - v0;
	A.row(2) = v3 - v0;
	return A.inverse() * B;
}

inline float determinantTetrahedron(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3)
{
	return (v1 - v0).cross(v2 - v0).dot(v3 - v0);
}

inline float determinantTriangle(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
	return (v1 - v0).cross(v2 - v1).dot(v0);
}

struct Tetrahedron 
{
	uint32_t IV0;
	uint32_t IV1;
	uint32_t IV2;
	uint32_t IV3;

	Eigen::Vector3f Circum;
	float CircumRadius2;

	std::array<Face, 4> Faces;

	inline Tetrahedron(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3, uint32_t iv0, uint32_t iv1, uint32_t iv2, uint32_t iv3)
		: IV0(iv0)
		, IV1(iv1)
		, IV2(iv2)
		, IV3(iv3)
		, Circum(circumCenterTetrahedron(v0, v1, v2, v3))
		, CircumRadius2((v0 - Circum).squaredNorm())
	{

		Faces[0] = Face{ IV1, IV2, IV3, nullptr, false };
		Faces[1] = Face{ IV0, IV3, IV2, nullptr, false };
		Faces[2] = Face{ IV0, IV1, IV3, nullptr, false };
		Faces[3] = Face{ IV0, IV1, IV2, nullptr, false };
	}

	inline bool contains(const Eigen::Vector3f& p) const
	{
		return (p - Circum).squaredNorm() <= CircumRadius2;
	}

	inline void makeBad()
	{
		IV1 = IV0;
		IV2 = IV0;
		IV3 = IV0;
	}

	inline bool isBad() const
	{
		return IV1 == IV0 && IV2 == IV0 && IV3 == IV0;
	}

	inline bool isSuper() const
	{
		return IV0 >= STP_START || IV1 >= STP_START || IV2 >= STP_START || IV3 >= STP_START;
	}

	inline Face* getAdjacentFace(const Face& f)
	{
		for (size_t j = 0; j < Faces.size(); ++j)
		{
			if (Faces[j] == f)
				return &Faces[j];
		}
		return nullptr;
	}

	inline void releaseNeighbor(size_t f)
	{
		releaseNeighbor(Faces[f]);
	}

	inline static void releaseNeighbor(Face& f)
	{
		if (f.Neighbor) 
		{
			auto ptr = f.Neighbor->getAdjacentFace(f);
			ptr->Neighbor = nullptr;
			f.Neighbor	  = nullptr;
		}
	}

	inline void setNeighbor(size_t f, Tetrahedron* t)
	{
		auto ptr = t->getAdjacentFace(Faces[f]);
		if (ptr) 
		{
			releaseNeighbor(f);
			releaseNeighbor(*ptr);

			Faces[f].Neighbor = t;
			ptr->Neighbor	  = this;
		}
	}

	inline void setupNeighbors(std::vector<TetrahedronPtr>& others)
	{
		for (size_t i = 0; i < Faces.size(); ++i) 
		{
			if (!Faces[i].Neighbor) 
			{
				for (auto it = others.begin(); it != others.end(); ++it) 
				{
					Tetrahedron* t = it->get();
					Face* f		   = t->getAdjacentFace(Faces[i]);
					if (f) 
					{
						t->releaseNeighbor(*f);
						Faces[i].Neighbor = t;
						f->Neighbor		  = this;
						break;
					}
				}
			}
		}
	}

	inline void disconnectFromNeighbors()
	{
		for (size_t i = 0; i < Faces.size(); ++i)
			releaseNeighbor(i);
	}
};

static void checkAndAdd(Tetrahedron* t, const Eigen::Vector3f& p, std::vector<Face>& aFaces)
{
	if (!t->isBad() && t->contains(p)) 
	{
		for (size_t j = 0; j < t->Faces.size(); ++j)
			aFaces.push_back(t->Faces[j]);
		t->makeBad();

		for (size_t j = 0; j < t->Faces.size(); ++j) 
		{
			if (t->Faces[j].Neighbor)
				checkAndAdd(t->Faces[j].Neighbor, p, aFaces);
		}
	}
}

inline static void addTetrahedron(std::vector<TetrahedronPtr>& list,
								  const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3,
								  uint32_t iv0, uint32_t iv1, uint32_t iv2, uint32_t iv3)
{
	auto ptr = std::make_unique<Tetrahedron>(v0, v1, v2, v3, iv0, iv1, iv2, iv3);
	ptr->setupNeighbors(list);
	list.emplace_back(std::move(ptr));
}

inline static void addTetrahedron(std::vector<TetrahedronPtr>& list, const Face& f,
								  const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& p,
								  /*iv0,iv1 and iv2 is given by f*/ uint32_t ip)
{
	auto ptr = std::make_unique<Tetrahedron>(v0, v1, v2, p, f.IV0, f.IV1, f.IV2, ip);

	if (f.Neighbor)
		ptr->setNeighbor(3, f.Neighbor);
	ptr->setupNeighbors(list);
	list.emplace_back(std::move(ptr));
}

std::vector<Triangle> triangulate3D(const std::vector<Eigen::Vector3f>& aVertices, bool surfaceOnly)
{
	if (aVertices.size() < 3)
		return std::vector<Triangle>();

	Eigen::Vector3f min = aVertices[0];
	Eigen::Vector3f max = min;
	for (const Eigen::Vector3f& p : aVertices) 
	{
		min = min.cwiseMin(p);
		max = max.cwiseMax(p);
	}

	max += Eigen::Vector3f::Ones();
	min -= Eigen::Vector3f::Ones();

	const Eigen::Vector3f stp0 = min;								// 000 (A)
	const Eigen::Vector3f stp1 = Eigen::Vector3f(max(0), min(1), min(2)); // 100 (B)
	const Eigen::Vector3f stp2 = Eigen::Vector3f(min(0), max(1), min(2)); // 010 (D)
	const Eigen::Vector3f stp3 = Eigen::Vector3f(max(0), max(1), min(2)); // 110 (C)
	const Eigen::Vector3f stp4 = Eigen::Vector3f(min(0), min(1), max(2)); // 001 (E)
	const Eigen::Vector3f stp5 = Eigen::Vector3f(max(0), min(1), max(2)); // 101 (F)
	const Eigen::Vector3f stp6 = Eigen::Vector3f(min(0), max(1), max(2)); // 011 (H)
	const Eigen::Vector3f stp7 = max;								// 111 (G)

#define _INIT_TETR(a, b, c, d) \
	addTetrahedron(sTetrahedrons, stp##a, stp##b, stp##c, stp##d, STP##a##_INDEX, STP##b##_INDEX, STP##c##_INDEX, STP##d##_INDEX)

	std::vector<TetrahedronPtr> sTetrahedrons;
	_INIT_TETR(0, 1, 3, 5); // ABCF
	_INIT_TETR(3, 6, 5, 7); // CHFG
	_INIT_TETR(0, 6, 3, 2); // AHCD
	_INIT_TETR(5, 0, 4, 6); // FAEH
	_INIT_TETR(0, 3, 6, 5); // ACHF

#undef _INIT_TETR

	const auto point = [&](size_t k) 
	{
		if ((k >= STP_START)) 
		{
			switch (k) 
			{
				case STP0_INDEX:
					return stp0;
				case STP1_INDEX:
					return stp1;
				case STP2_INDEX:
					return stp2;
				case STP3_INDEX:
					return stp3;
				case STP4_INDEX:
					return stp4;
				case STP5_INDEX:
					return stp5;
				case STP6_INDEX:
					return stp6;
				case STP7_INDEX:
					return stp7;
				default:
					return aVertices[k];
			}
		} 
		else 
		{
			return aVertices[k];
		}
	};

	std::vector<Face> sFaces; 
	for (size_t i = 0; i < aVertices.size(); ++i) 
	{
		const Eigen::Vector3f p = aVertices[i];

		sFaces.clear();
		for (auto it = sTetrahedrons.begin(); it != sTetrahedrons.end(); ++it) 
		{
			Tetrahedron* t = it->get();
			if (t->contains(p)) 
			{
				for (size_t j = 0; j < t->Faces.size(); ++j)
					sFaces.push_back(t->Faces[j]);
				t->makeBad();

				for (size_t j = 0; j < t->Faces.size(); ++j) 
				{
					if (t->Faces[j].Neighbor)
						checkAndAdd(t->Faces[j].Neighbor, p, sFaces);
				}
				break;
			}
		}

		for (auto it = sTetrahedrons.begin(); it != sTetrahedrons.end();) 
		{
			if (it->get()->isBad()) 
			{
				it->get()->disconnectFromNeighbors();
				it = sTetrahedrons.erase(it);
			} 
			else 
			{
				++it;
			}
		}

		for (auto it1 = sFaces.begin(); it1 != sFaces.end(); ++it1) 
		{
			if (it1->isBad())
				continue;
			for (auto it2 = it1 + 1; it2 != sFaces.end(); ++it2) 
			{
				if (*it1 == *it2) 
				{
					it1->makeBad();
					it2->makeBad();
				}
			}
		}

		for (const auto& f : sFaces) 
		{
			if (!f.isBad())
				addTetrahedron(sTetrahedrons, f, point(f.IV0), point(f.IV1), point(f.IV2), p, i);
		}
	}
	std::vector<Face>().swap(sFaces);

	for (auto it = sTetrahedrons.begin(); it != sTetrahedrons.end();) 
	{
		if (it->get()->isBad() || it->get()->isSuper()) 
		{
			it->get()->disconnectFromNeighbors();
			it = sTetrahedrons.erase(it);
		} 
		else 
		{
			++it;
		}
	}

	std::vector<Triangle> sTriangles;
	const auto addFace = [&](const Face& f) { sTriangles.push_back(Triangle{ f.IV0, f.IV1, f.IV2 }); };

	if (surfaceOnly) 
	{
		for (auto it = sTetrahedrons.begin(); it != sTetrahedrons.end(); ++it) 
		{
			Tetrahedron* t = it->get();
			for (size_t j = 0; j < t->Faces.size(); ++j) {
				if (!t->Faces[j].Neighbor)
					addFace(t->Faces[j]);
			}
		}
	} 
	else 
	{
		std::unordered_set<Face, FaceHash> sUniqueFaces;
		for (auto it = sTetrahedrons.begin(); it != sTetrahedrons.end(); ++it) 
		{
			Tetrahedron* t = it->get();
			for (size_t j = 0; j < t->Faces.size(); ++j)
				sUniqueFaces.insert(t->Faces[j]);
		}

		for (const auto& f : sUniqueFaces)
			addFace(f);
	}
	return sTriangles;
}
} // namespace Triangulation
