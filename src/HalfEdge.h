#pragma once
#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <Eigen\Dense>

class HalfEdge {
	//struct HE_halfedge;
public:
	struct HE_vert {
		Eigen::Vector3f pos;
		//HE_halfedge* halfedge;
	};
	struct HE_halfedge {
		int v_idx;
		int f_idx;
		int e_idx;
		int pair;
		int next;
		float angel;
		bool isSwaped;
		HE_halfedge() {
			pair = -1;
			isSwaped = false;
		}
	};
	struct HE_face {
		int e_idx;
		int edge2swap;
		HE_face() {
			edge2swap = 0;
		}
	};
public:
	HalfEdge();
	~HalfEdge();
	void addVert(float x, float y, float z);
	//void addEdge(int v1, int v2);
	void addFace(int i1, int i2, int i3);
	void swapEdges();
	void swapOneEdge(HE_halfedge* e);
	std::vector<HE_face*> &getFaces();
	std::vector<HE_halfedge*> &getEdges();
	void swapp(HE_halfedge*& rp1, HE_halfedge*& rp2);
	//void func();
private:
	std::vector<HE_vert*> m_verts_;
	std::vector<HE_halfedge*> m_edges_;
	std::vector<HE_face*> m_faces_;
	std::map<std::pair<int, int>, HE_halfedge*> Edges_;
	float swap_angel_;
};