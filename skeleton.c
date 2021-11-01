#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

#include <math.h>
#include "inc/bcl/bmap.c"
#include "inc/linalg.h"
#include "inc/linuxfb.h"
#include "inc/lfb2d.h"
#include "inc/mouse.h"

struct ik_node {
	double length;
	double aperture;
	vec3f pos;
	vec3f rtn;
};

struct ik_chain {
	struct ik_node* nodes;
	short num_nodes;
};

int ik_draw_chain(struct ik_chain* _chain, int, int);

/* solves an inverse kinematics chain for the specified target using
 * the method of cyclic coordinate descent (CCD). (returns -1 on error) */
int ik_chain_solve_ccd(struct ik_chain* _chain, vec3f _target, double _rigidity) {
	vec3f effector_vec; // vector from bone pivot to effector
	vec3f target_vec; // vector from bone pivot to target
	
	double phi;
	
	for (int i = 0; i < 10; i++) // number of CCD iterations
	for (short k = 1; k < _chain->num_nodes; k++) {
		effector_vec = sub3f(_chain->nodes[0].pos, _chain->nodes[k].pos);
		target_vec = sub3f(_target, _chain->nodes[k].pos);
		
		// calculate rotation needed to face joint towards the target
		double denom = (mag3f(effector_vec) * mag3f(target_vec));
		double cos_phi = dot3f(effector_vec, target_vec) / denom;
		
		// return if `denom` is 0 (some node likely has a zero vector
		// for its direction)
		if (fabs(denom) < 0.0000000001) return -1;
		
		// prevent acos from giving nan and causing the entire code-
		// base to spiral into chaos (even with 1.0000000000001).
		if (cos_phi > 0.999999999) phi = 0;
		else if (cos_phi < -0.999999999) phi = PI;
		else phi = acos_that_actually_works(cos_phi);
		
		// smoothing coefficient
		phi *= _rigidity;
		
		// rotate joint and update remaining chain
		vec3f axis = norm3f(cross3f(effector_vec, target_vec));
		qtrn rot = make_qrot(axis, phi);
		
		_chain->nodes[k].rtn = norm3f(qrot(rot, _chain->nodes[k].rtn));
		
		for (int n = k - 1; n >= 0; n--) {
			_chain->nodes[n].rtn = norm3f(qrot(rot, _chain->nodes[n].rtn));
			_chain->nodes[n].pos.x = _chain->nodes[n + 1].pos.x + _chain->nodes[n + 1].length * _chain->nodes[n + 1].rtn.x;
			_chain->nodes[n].pos.y = _chain->nodes[n + 1].pos.y + _chain->nodes[n + 1].length * _chain->nodes[n + 1].rtn.y;
			_chain->nodes[n].pos.z = _chain->nodes[n + 1].pos.z + _chain->nodes[n + 1].length * _chain->nodes[n + 1].rtn.z;
		}
	}
	
	return 0;
}

/* solves an inverse kinematics chain for the specified target using
 * the method of forward and backward reaching inverse kinematics (FABRIK). */
int ik_chain_solve_fabrik(struct ik_chain* _chain, vec3f _target) {
	vec3f target; // current target position (not always `_target`)
	vec3f joint_vec; // vector from joint to current target
	vec3f root = _chain->nodes[_chain->num_nodes - 1].pos; // root position of chain
	
	double cos_phi;
	
	// one hundred FABRIK iterations
	for (int k, i = 0; i < 1; i++) {
		// forward pass
		target = _chain->nodes[0].pos = _target;
		
		for (k = 1; k < _chain->num_nodes - 1; k++) {
			joint_vec = norm3f(sub3f(target, _chain->nodes[k].pos));
			
			// check if joint is within limits
			cos_phi = dot3f(_chain->nodes[k - 1].rtn, joint_vec);
			
			if (acos(cos_phi) > _chain->nodes[k - 1].aperture) {
				vec3f axis = norm3f(cross3f(_chain->nodes[k - 1].rtn, joint_vec));
				qtrn q = make_qrot(axis, _chain->nodes[k - 1].aperture);
				joint_vec = qrot(q, _chain->nodes[k - 1].rtn);
			}
			
			target = sub3f(target, mul3f(joint_vec, _chain->nodes[k].length));
			_chain->nodes[k].pos = target;
			_chain->nodes[k].rtn = joint_vec;
		}
		
		// backward pass
		target = _chain->nodes[_chain->num_nodes - 1].pos = root;
		joint_vec = _chain->nodes[_chain->num_nodes - 1].rtn;
		
		for (k = _chain->num_nodes - 2; k >= 0; k--) {
			_chain->nodes[k + 1].rtn = joint_vec;
			joint_vec = norm3f(sub3f(target, _chain->nodes[k].pos));
			
			// check if joint is within limits
			cos_phi = dot3f(_chain->nodes[k + 1].rtn, joint_vec);
			
			if (acos(cos_phi) > _chain->nodes[k + 1].aperture) {
				vec3f axis = norm3f(cross3f(_chain->nodes[k + 1].rtn, joint_vec));
				qtrn q = make_qrot(axis, _chain->nodes[k + 1].aperture);
				joint_vec = qrot(q, _chain->nodes[k + 1].rtn);
			}
			
			target = sub3f(target, mul3f(joint_vec, _chain->nodes[k + 1].length));
			_chain->nodes[k].pos = target;
		}
	}
	
	return 0;
}

int ik_make_chain(struct ik_chain* _chain, short _num_nodes, double _length) {
	_chain->num_nodes = _num_nodes + 1;
	
	_chain->nodes = malloc(_chain->num_nodes * sizeof(struct ik_node));
	if (!_chain->nodes) return -1;
	
	_chain->nodes[_num_nodes].length = _length;
	_chain->nodes[_num_nodes].aperture = PI;
	_chain->nodes[_num_nodes].rtn = (vec3f) {1, 0, 0};
	_chain->nodes[_num_nodes].pos = (vec3f) {0, 0, 0};
	
	for (int i = _num_nodes - 1; i > 0; i--) {
		_chain->nodes[i].length = _length;
		_chain->nodes[i].aperture = PI/6;
		_chain->nodes[i].rtn = (vec3f) {1, 0, 0};
		_chain->nodes[i].pos.x = _chain->nodes[i + 1].pos.x + _chain->nodes[i + 1].length;
		_chain->nodes[i].pos.y = _chain->nodes[i].pos.z = 0;
	}
	
	_chain->nodes[0].length = 0;
	_chain->nodes[0].aperture = PI;
	_chain->nodes[0].rtn = (vec3f) {1, 0, 0};
	_chain->nodes[0].pos.x = _chain->nodes[1].pos.x + _chain->nodes[1].length;
	_chain->nodes[0].pos.y = _chain->nodes[0].pos.z = 0;
	
	return 0;
}

int ik_reset_chain(struct ik_chain* _chain) {
	_chain->nodes[_chain->num_nodes - 1].rtn = (vec3f) {1, 0, 0};
	_chain->nodes[_chain->num_nodes - 1].pos = (vec3f) {0, 0, 0};
	
	for (int i = _chain->num_nodes - 1; i > 0; i--) {
		_chain->nodes[i].rtn = _chain->nodes[i].rtn = (vec3f) {1, 0, 0};
		_chain->nodes[i].pos.x = _chain->nodes[i + 1].pos.x + _chain->nodes[i + 1].length;
		_chain->nodes[i].pos.y = _chain->nodes[i].pos.z = 0;
	}
	
	_chain->nodes[0].rtn = (vec3f) {1, 0, 0};
	_chain->nodes[0].pos.x = _chain->nodes[1].pos.x + _chain->nodes[1].length;
	_chain->nodes[0].pos.y = _chain->nodes[0].pos.z = 0;
	
	return 0;
}

int ik_draw_chain(struct ik_chain* _chain, int _x_off, int _y_off) {
	for (int i = 1; i < _chain->num_nodes; i++) {
		fb_draw_line((rgbx32) {255, 255, 255, 255}, _chain->nodes[i].pos.x + _x_off, _chain->nodes[i].pos.y + _y_off, _chain->nodes[i - 1].pos.x + _x_off, _chain->nodes[i - 1].pos.y + _y_off);
		//fb_draw_line((rgbx32) {50, 25, 100, 255}, _chain->nodes[i].pos.x + _x_off, _chain->nodes[i].pos.y + _y_off, _chain->nodes[i].pos.x + _x_off + _chain->nodes[i].rtn.x * 10, _chain->nodes[i].pos.y + _y_off + _chain->nodes[i].rtn.y * 10);
	}
	
	for (int i = 1; i < _chain->num_nodes; i++)
		fb_draw_point((rgbx32) {255, 255, 255, 255}, _chain->nodes[i].pos.x + _x_off, _chain->nodes[i].pos.y + _y_off);
	
	return 0;
}

int main(void) {
	fb_init("/dev/fb0");
	mouse_init("/dev/input/mice");
	
	struct ik_chain n1;
	vec3f t = {-100.0, 100.0, 0.0};
	ik_make_chain(&n1, 100, 5);
	
	mouse_event ev;
	
	for (;;) {
		ev = get_mouse_event();
		
		if (ev.lbtn) t.z += 10.0;
		if (ev.rbtn) t.z -= 10.0;
		
		t.x += (double) ev.rel_x;
		t.y -= (double) ev.rel_y;
		
//		ik_reset_chain(&n1);
		ik_chain_solve_fabrik(&n1, t);
		ik_draw_chain(&n1, 200, 200);
		fb_draw_centroid((rgbx32) {255, 255, 0, 255}, t.x + 200, t.y + 200);
		fb_swap();
	}
}

#endif