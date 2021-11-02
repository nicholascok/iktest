#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

enum {ERROR_FAILED_ALLOCATE};

#define XERR(MESG, ERRCODE) { perror(MESG"\n"); exit(ERRCODE); }

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
	double aperture;
	vec3f rtn;
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

/* internal function that clamps a vector to the inside of a cone: the cone's
 * axis is a vector from its tip to the centre of its base, and the cone's aperture
 * is the angle between its axis and its edge. (both vectors must be normalised) */
vec3f __ik_clamp_vector_to_cone(vec3f _cone_axis, double _cone_aperture, vec3f _v) {
		// we are assuming the provided vectors are normalised so the
		// cosine of the angle between them is simply their dot product
		double cos_phi = dot3f(_cone_axis, _v);
		
		// if the angle between the cone axis and our vector is greater than
		// the aperture of the cone then the vector needs to be clamped
		if (acos(cos_phi) > _cone_aperture) {
			// rotate the cone's axis to its edge about an axis orthogonal
			// to both the axis of the cone and the vector we wish to clamp
			// (with quaternions!)
			vec3f axis = norm3f(cross3f(_cone_axis, _v));
			qtrn q = make_qrot(axis, _cone_aperture);
			return qrot(q, _cone_axis);
		}
		
		// if the vector was not clamped, return it unchanged
		return _v;
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
		///////////////////////////////////////////
		// FORWARD PASS                          //
		///////////////////////////////////////////
		target = _chain->nodes[0].pos = _target;
		
		// proces sinitial joint
		joint_vec = norm3f(sub3f(target, _chain->nodes[1].pos));
		target = sub3f(target, mul3f(joint_vec, _chain->nodes[0].length));
		_chain->nodes[1].pos = target;
		_chain->nodes[1].rtn = joint_vec;
		
		for (k = 1; k < _chain->num_nodes - 2; k++) {
			joint_vec = norm3f(sub3f(target, _chain->nodes[k + 1].pos));
			
			// apply angle restrictions
			joint_vec = __ik_clamp_vector_to_cone(
				_chain->nodes[k].rtn,
				_chain->nodes[k].aperture,
				joint_vec
			);
			
			target = sub3f(target, mul3f(joint_vec, _chain->nodes[k].length));
			_chain->nodes[k + 1].pos = target;
			_chain->nodes[k + 1].rtn = joint_vec;
		}
		
		///////////////////////////////////////////
		// BACKWARD PASS                         //
		///////////////////////////////////////////
		target = _chain->nodes[_chain->num_nodes - 1].pos = root;
		
		// process initial joint
		joint_vec = norm3f(sub3f(_chain->nodes[_chain->num_nodes - 2].pos, target));
		joint_vec = __ik_clamp_vector_to_cone(_chain->rtn, _chain->aperture, joint_vec);
		target = add3f(target, mul3f(joint_vec, _chain->nodes[_chain->num_nodes - 1].length));
		_chain->nodes[_chain->num_nodes - 2].pos = target;
		_chain->nodes[_chain->num_nodes - 1].rtn = joint_vec;
		
		// process remaining joints
		for (k = _chain->num_nodes - 2; k > 0; k--) {
			joint_vec = norm3f(sub3f(_chain->nodes[k - 1].pos, target));
			
			// apply angle restrictions
			joint_vec = __ik_clamp_vector_to_cone(
				_chain->nodes[k + 1].rtn,
				_chain->nodes[k + 1].aperture,
				joint_vec
			);
			
			target = add3f(target, mul3f(joint_vec, _chain->nodes[k].length));
			_chain->nodes[k - 1].pos = target;
			_chain->nodes[k].rtn = joint_vec;
		}
		
		// set effector rotation
		_chain->nodes[0].rtn = _chain->nodes[1].rtn;
	}
	
	return 0;
}

int ik_make_chain(struct ik_chain* _chain, short _num_nodes, double _length) {
	_chain->num_nodes = _num_nodes + 1;
	_chain->aperture = PI/2;
	_chain->rtn = (vec3f) {1, 0, 0};
	
	_chain->nodes = malloc(_chain->num_nodes * sizeof(struct ik_node));
	if (!_chain->nodes) XERR("failed to allocate memory!", ERROR_FAILED_ALLOCATE);
	
	for (int i = _num_nodes; i > 0; i--) {
		_chain->nodes[i].length = _length;
		_chain->nodes[i].aperture = PI/6;
		_chain->nodes[i].rtn = (vec3f) {1, 0, 0};
		_chain->nodes[i].pos.x = _chain->nodes[i + 1].pos.x + _chain->nodes[i + 1].length;
		_chain->nodes[i].pos.y = _chain->nodes[i].pos.z = 0;
	}
	
	_chain->nodes[0].length = 0;
	_chain->nodes[0].aperture = 0;
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
		// draw bone
		fb_draw_line(
			(rgbx32) {255, 255, 255, 255},
			_chain->nodes[i].pos.x + _x_off,
			_chain->nodes[i].pos.y + _y_off,
			_chain->nodes[i - 1].pos.x + _x_off,
			_chain->nodes[i - 1].pos.y + _y_off
		);
		
		// draw direction
		fb_draw_line(
			(rgbx32) {255, 255, 255,255},
			_chain->nodes[i].pos.x + _x_off,
			_chain->nodes[i].pos.y + _y_off,
			_chain->nodes[i - 1].pos.x + _x_off,
			_chain->nodes[i - 1].pos.y + _y_off
		);
	}
	
	// draw effector direction
	fb_draw_line(
		(rgbx32) {0, 0, 255, 255},
		_chain->nodes[0].pos.x + _x_off,
		_chain->nodes[0].pos.y + _y_off,
		_chain->nodes[0].pos.x + _x_off + _chain->nodes[0].rtn.x * 10,
		_chain->nodes[0].pos.y + _y_off + _chain->nodes[0].rtn.y * 10
	);
	
	// draw joint points
	for (int i = 0; i < _chain->num_nodes; i++) {
		fb_draw_point(
			(rgbx32) {255, 255, 255, 255},
			_chain->nodes[i].pos.x + _x_off,
			_chain->nodes[i].pos.y + _y_off
		);
	}
	
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