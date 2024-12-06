#include "HalfEdge.h"
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif // !M_PI
HalfEdge::HalfEdge()
{
	swap_angel_ = 120.0f;
}

HalfEdge::~HalfEdge()
{
	for (std::vector<HE_vert*>::iterator it = m_verts_.begin(); it != m_verts_.end(); it++) {
		if (*it != NULL) {
			delete *it;
			*it = NULL;
		}
	}
	for (std::vector<HE_halfedge*>::iterator it = m_edges_.begin(); it != m_edges_.end(); it++) {
		if (*it != NULL) {
			delete *it;
			*it = NULL;
		}
	}
	for (std::vector<HE_face*>::iterator it = m_faces_.begin(); it != m_faces_.end(); it++) {
		if (*it != NULL) {
			delete *it;
			*it = NULL;
		}
	}
	/*for (std::map<std::pair<int, int>, HE_halfedge*>::iterator it = Edges_.begin(); it != Edges_.end();) {
	if (it->second != NULL) {
	delete it->second;
	Edges_.erase(it++);
	}
	}*/
}

void HalfEdge::addVert(float x, float y, float z)
{
	HE_vert* vert = new HE_vert();
	vert->pos = Eigen::Vector3f(x, y, z);
	m_verts_.emplace_back(vert);
}


void HalfEdge::addFace(int i1, int i2, int i3)
{
	HE_face* f = new HE_face();
	HE_halfedge* e1;
	HE_halfedge* e2;
	HE_halfedge* e3;
	m_faces_.emplace_back(f);
	std::pair<int, int> uv = (std::make_pair(i1, i2));
	std::pair<int, int> vu = (std::make_pair(i2, i1));
	if (Edges_.find(uv) != Edges_.end()) {
		printf("repeated edge\n");
		system("pause");
	}
	e1 = new HE_halfedge();
	e1->e_idx = m_edges_.size();
	Edges_[uv] = e1;
	e1->v_idx = i1;
	e1->f_idx = m_faces_.size() - 1;
	m_edges_.emplace_back(e1);
	if (Edges_.find(vu) != Edges_.end()) {
		Edges_[uv]->pair = Edges_[vu]->e_idx;
		Edges_[vu]->pair = e1->e_idx;
	}

	uv = (std::make_pair(i2, i3));
	vu = (std::make_pair(i3, i2));
	if (Edges_.find(uv) != Edges_.end()) {
		printf("repeated edge\n");
		system("pause");
	}
	e2 = new HE_halfedge();
	e2->e_idx = m_edges_.size();
	Edges_[uv] = e2;
	e2->v_idx = i2;
	e2->f_idx = m_faces_.size() - 1;
	//e2->next = e3;
	m_edges_.emplace_back(e2);
	if (Edges_.find(vu) != Edges_.end()) {
		Edges_[uv]->pair = Edges_[vu]->e_idx;
		Edges_[vu]->pair = e2->e_idx;
	}

	uv = (std::make_pair(i3, i1));
	vu = (std::make_pair(i1, i3));
	if (Edges_.find(uv) != Edges_.end()) {
		printf("repeated edge\n");
		system("pause");
	}
	e3 = new HE_halfedge();
	e3->e_idx = m_edges_.size();
	Edges_[uv] = e3;
	e3->v_idx = i3;
	e3->f_idx = m_faces_.size() - 1;
	//e3->next = e1;
	m_edges_.emplace_back(e3);
	if (Edges_.find(vu) != Edges_.end()) {
		Edges_[uv]->pair = Edges_[vu]->e_idx;
		Edges_[vu]->pair = e3->e_idx;
	}

	e1->next = e2->e_idx;
	e2->next = e3->e_idx;
	e3->next = e1->e_idx;
	f->e_idx = e1->e_idx;

	Eigen::Vector3f v1v2 = (m_verts_[i2]->pos - m_verts_[i1]->pos).normalized();
	Eigen::Vector3f v2v3 = (m_verts_[i3]->pos - m_verts_[i2]->pos).normalized();
	Eigen::Vector3f v3v1 = (m_verts_[i1]->pos - m_verts_[i3]->pos).normalized();
	e2->angel = acos(-v1v2.transpose()*v3v1) / M_PI*180.0f;
	e3->angel = acos(-v1v2.transpose()*v2v3) / M_PI*180.0f;
	e1->angel = acos(-v2v3.transpose()*v3v1) / M_PI*180.0f;
	//printf("Angel:%d %f %f %f\n", e1->f_idx, e1->angel, e2->angel, e3->angel);
	//printf("v:%d %d %d\n", e1->v_idx, e2->v_idx, e3->v_idx);
	if (e1->angel > swap_angel_)
		f->edge2swap = 1;
	else if (e2->angel > swap_angel_)
		f->edge2swap = 2;
	else if (e3->angel > swap_angel_)
		f->edge2swap = 3;
}

void HalfEdge::swapEdges()
{
	for (int i = 0; i < m_faces_.size(); i++) {
		if (m_faces_[i]->edge2swap != 0) {
			HE_halfedge* edge = m_edges_[m_faces_[i]->e_idx];
			for (int j = 1; j < m_faces_[i]->edge2swap; j++)
				edge = m_edges_[edge->next];
			swapOneEdge(edge);
		}
	}
}

void HalfEdge::swapOneEdge(HE_halfedge * e)
{
	//printf("ID:%d\n", e->f_idx);
	if (e->isSwaped == true||e->pair == -1) {
		//printf("v:%d %d %d\n", e->v_idx, e->next->v_idx, e->next->next->v_idx);
		//printf("a:%f %f %f\n", e->angel, e->next->angel, e->next->next->angel);
		return;
	}
	//return;
	e->isSwaped = true;
	HE_face* f = m_faces_[e->f_idx];
	HE_halfedge* e_pair = m_edges_[e->pair];
	HE_face* f_pair = m_faces_[e_pair->f_idx];
	HE_halfedge* e2 = m_edges_[e->next];
	HE_halfedge* e3 = m_edges_[e2->next];
	HE_halfedge* e_pair2 = m_edges_[e_pair->next];
	HE_halfedge* e_pair3 = m_edges_[e_pair2->next];
	//printf("a:%f %f %f\n", e->angel, e2->angel, e3->angel);
	//printf("e:%d %d %d\n", e->v_idx, e2->v_idx, e3->v_idx);
	if (f_pair->edge2swap != 0) {
		HE_halfedge* edge = m_edges_[f_pair->e_idx];
		for (int i = 1; i < f_pair->edge2swap; i++)
			edge = m_edges_[edge->next];
		if (edge != e_pair)
			swapOneEdge(edge);
	}
	else if (e2->angel + e_pair3->angel > swap_angel_&&e_pair3->angel > swap_angel_ / 2) {
		HE_halfedge* edge = e_pair3;
		swapOneEdge(edge);
	}
	else if (e3->angel + e_pair2->angel > swap_angel_&&e_pair2->angel > swap_angel_ / 2) {
		HE_halfedge* edge = e_pair2;
		swapOneEdge(edge);
	}
	e_pair = m_edges_[e->pair];
	e_pair2 = m_edges_[e_pair->next];
	e_pair3 = m_edges_[e_pair2->next];
	f_pair = m_faces_[e_pair->f_idx];
	e->v_idx = e_pair3->v_idx;
	e->angel = e2->angel + e_pair3->angel;
	e->next = e3->e_idx;
	e3->next = e_pair2->e_idx;
	e_pair2->next = e->e_idx;
	e_pair2->f_idx = e->f_idx;
	float temp_angel = e3->angel;
	/*HE_halfedge* temp = new HE_halfedge();
	temp->v_idx = e2->v_idx;
	temp->pair = e2->pair;
	temp->isSwaped = e2->isSwaped;
	float temp_angel = e3->angel;
	e2->v_idx = e3->v_idx;
	e2->pair = e3->pair;
	e2->isSwaped = e3->isSwaped;
	e3->v_idx = e_pair2->v_idx;
	e3->pair = e_pair2->pair;
	e3->isSwaped = e_pair2->isSwaped;
	e3->next = e->e_idx;
	e3->f_idx = e->f_idx;*/
	Eigen::Vector3f v1v2 = (m_verts_[e3->v_idx]->pos - m_verts_[e->v_idx]->pos).normalized();
	Eigen::Vector3f v2v3 = (m_verts_[e->v_idx]->pos - m_verts_[e_pair2->v_idx]->pos).normalized();
	e3->angel = acos(-v1v2.transpose()*v2v3) / M_PI*180.0f;
	e_pair2->angel = 180.0f - e->angel - e3->angel;
	//e->isSwaped = true;
	//printf("a:%f %f %f\n", e->angel, e->next->angel, e->next->next->angel);

	//printf("pair_v_origin:%d %d %d\n", e_pair->v_idx, e_pair->next->v_idx, e_pair->next->next->v_idx);
	e_pair->v_idx = e3->v_idx;
	e_pair->angel = temp_angel + e_pair2->angel;
	e_pair->next = e_pair3->e_idx;
	e_pair3->next = e2->e_idx;
	e2->next = e_pair->e_idx;
	e2->f_idx = e_pair->f_idx;

	//e_pair2->pair = e_pair3->pair;
	//e_pair2->isSwaped = e_pair3->isSwaped;
	////printf("f1:%d %d %d\n", e->v_idx, e->next->v_idx, e->next->next->v_idx);
	//e_pair3->v_idx = temp->v_idx;
	//e_pair3->pair = temp->pair;
	//e_pair3->isSwaped = temp->isSwaped;
	//e_pair3->next = e_pair->e_idx;
	////printf("f2:%d %d %d\n", e->v_idx, e->next->v_idx, e->next->next->v_idx);
	//e_pair3->f_idx = e_pair->f_idx;
	v1v2 = (m_verts_[e_pair->v_idx]->pos - m_verts_[e2->v_idx]->pos).normalized();
	v2v3 = (m_verts_[e_pair3->v_idx]->pos - m_verts_[e_pair->v_idx]->pos).normalized();
	e_pair3->angel = acos(-v1v2.transpose()*v2v3) / M_PI*180.0f;
	e_pair2->angel = 180.0f - e_pair->angel - e_pair3->angel;
	e_pair->isSwaped = true;
	//printf("v:%d %d %d\n", e->v_idx, e3->v_idx, e_pair2->v_idx);
	//printf("pair_v:%d %d %d\n", e_pair->v_idx, e_pair3->v_idx, e2->v_idx);
	//printf("v_f:%d %d %d\n", e->f_idx, e3->f_idx, e_pair2->f_idx);
	//printf("pair_v_f:%d %d %d\n", e_pair->f_idx, e_pair3->f_idx, e2->f_idx);
	//printf("a:%f %f %f\n", e_pair->angel, e_pair->next->angel, e_pair->next->next->angel);

	f->e_idx = e->e_idx;
	f->edge2swap = 0;
	f_pair->e_idx = e_pair->e_idx;
	f_pair->edge2swap = 0;
	//printf("f:%d %d %d\n", e->v_idx, e2->v_idx, e3->v_idx);
	/*HE_face* f_ = m_faces_[2];
	HE_halfedge* e_ = m_edges_[f->e_idx];
	HE_halfedge* e_2 = m_edges_[e_->next];
	HE_halfedge* e_3 = m_edges_[e_2->next];
	printf("f2v:%d %d %d\n", e_->v_idx, e_2->v_idx, e_3->v_idx);*/
}

std::vector<HalfEdge::HE_face *>& HalfEdge::getFaces()
{
	return m_faces_;
}

std::vector<HalfEdge::HE_halfedge*>& HalfEdge::getEdges()
{
	return m_edges_;
}

void HalfEdge::swapp(HE_halfedge *& rp1, HE_halfedge *& rp2)
{
	HE_halfedge* t;
	t = rp1; rp1 = rp2; rp2 = t;
}

//void HalfEdge::func()
//{
//	for (int i = 0; i < m_faces_.size(); i++) {
//
//		HE_face *f = m_faces_[i];
//		HE_halfedge *e = f->halfedge;
//		/*printf("0000\n");
//		faces_[i].x = e->v_idx;
//		printf("1111\n");
//		faces_[i].y = e->next->v_idx;
//		printf("2222\n");
//		faces_[i].z = e->next->next->v_idx;*/
//		printf("%d  %d  %d \n", e->v_idx, e->next->v_idx, e->next->v_idx);
//	}
//}




