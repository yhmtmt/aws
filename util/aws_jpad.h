// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// aws_jpad.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_jpad.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_jpad.h.  If not, see <http://www.gnu.org/licenses/>. 


#ifndef _AWS_JPAD_H_
#define _AWS_JPAD_H_
//s_jc_u3613m  Joystick handling structure
struct s_jc_u3613m
{
	int id;
	const char * name;
	int naxs, nbtn;
	enum e_btn{
		EB_EVUP = 0x1, EB_EVDOWN = 0x2, EB_STUP = 0x4, EB_STDOWN = 0x8
	};

	float thstk; // stick response threashold
	float lr1, ud1, lr2, ud2;
	unsigned char eux, erx, edx, elx;
	int tux, trx, tdx, tlx;

	int tx, ty, ta, tb, tlb, trb, tlt, tlst, trst, trt, tback, tstart, tguide; // cycle time of the button pressed
	unsigned char ex, ey, ea, eb, elb, erb, elt, elst, erst, ert, eback, estart, eguide; // button event flags

	s_jc_u3613m() :
		id(-1), thstk(0.2f), eux(0), erx(0), edx(0), elx(0), tux(0), trx(0), tdx(0), tlx(0),
		tx(0), ty(0), ta(0), tb(0), tlb(0), trb(0), tlt(0), tlst(0),
		trst(0), trt(0), tback(0), tstart(0), tguide(0),
		ex(0), ey(0), ea(0), eb(0), elb(0), erb(0), elt(0), elst(0),
		erst(0), ert(0), eback(0), estart(0), eguide(0)
	{
	}

	bool init(int _id){
		if (glfwJoystickPresent(_id) == GL_TRUE){
			id = _id;
			name = glfwGetJoystickName(_id);
			return true;
		}
		id = -1;
		return false;
	}

	void set_stk(){
		const float * axs = glfwGetJoystickAxes(id, &naxs);
		lr1 = set_stk(axs[0]);
		ud1 = set_stk(axs[1]);
		lr2 = set_stk(axs[2]);
		ud2 = set_stk(axs[3]);

		if (naxs > 4){ // may be in linux
			erx = set_btn(axs[4] > 0.5 ? 1 : 0, erx);
			elx = set_btn(axs[4] < -0.5 ? 1 : 0, elx);
			edx = set_btn(axs[5] > 0.5 ? 1 : 0, edx);
			eux = set_btn(axs[5] < -0.5 ? 1 : 0, eux);
			trx = set_tbtn(erx, trx);
			tlx = set_tbtn(elx, tlx);
			tux = set_tbtn(eux, tux);
			tdx = set_tbtn(edx, tdx);
		}
	}

	const float set_stk(const float v){
		if (v < -thstk){
			return v + thstk;
		}
		else if (v > thstk){
			return v - thstk;
		}
		return 0;
	}

	void set_btn()
	{
		const unsigned char * btn = glfwGetJoystickButtons(id, &nbtn);
		ex = set_btn(btn[0], ex);
		ey = set_btn(btn[1], ey);
		ea = set_btn(btn[2], ea);
		eb = set_btn(btn[3], eb);
		elb = set_btn(btn[4], elb);
		erb = set_btn(btn[5], erb);
		elt = set_btn(btn[6], elt);
		ert = set_btn(btn[7], ert);
		elst = set_btn(btn[8], elst);
		erst = set_btn(btn[9], erst);
		eback = set_btn(btn[10], eback);
		estart = set_btn(btn[11], estart);
		eguide = set_btn(btn[12], eguide);
		tx = set_tbtn(ex, tx);
		ty = set_tbtn(ey, ty);
		ta = set_tbtn(ea, ta);
		tb = set_tbtn(eb, tb);
		tlb = set_tbtn(elb, tlb);
		trb = set_tbtn(erb, trb);
		tlt = set_tbtn(elt, tlt);
		tlst = set_tbtn(elst, tlst);
		trst = set_tbtn(erst, trst);
		trt = set_tbtn(ert, trt);
		tback = set_tbtn(eback, tback);
		tstart = set_tbtn(estart, tstart);
		tguide = set_tbtn(eguide, tguide);
		if (nbtn > 13){ // maybe windows case
			// U13, R14, D15, L16
			erx = set_btn(btn[14], erx);
			elx = set_btn(btn[16], elx);
			eux = set_btn(btn[13], eux);
			edx = set_btn(btn[15], edx);

			trx = set_tbtn(erx, trx);
			tlx = set_tbtn(elx, tlx);
			tux = set_tbtn(eux, tux);
			tdx = set_tbtn(edx, tdx);
		}
	}

	unsigned char set_btn(unsigned char vnew, unsigned char eold)
	{
		return
			/* 1st bit event up */(((EB_STDOWN & eold) >> 3) & ~vnew) |
			/* 2nd bit event down */ ((((EB_STUP & eold) >> 2) & vnew) << 1) |
			/* 3rd bit state up */  ((0x1 & ~vnew) << 2) |
			/* 3rd bit state down */ (vnew << 3);
	}

	int set_tbtn(unsigned char enew, int told){
		if (EB_EVUP & enew)
			return 0;
		return (EB_STDOWN & enew) ? told + 1 : told;
	}

	void print(ostream & out)
	{
		out << "STK>>";
		out << "LR1:" << lr1 << " UD1:" << ud1 << " ";
		out << "LR2:" << lr2 << " UD2:" << ud2 << " ";
		print(out, "LX", elx, tlx); out << " ";
		print(out, "RX", erx, trx); out << " ";
		print(out, "UX", eux, tux); out << " ";
		print(out, "DX", edx, tdx);
		out << endl;

		out << "BTN>>";
		print(out, "x", ex, tx); out << " ";
		print(out, "y", ey, ty); out << " ";
		print(out, "a", ea, ta); out << " ";
		print(out, "b", eb, tb); out << " ";
		print(out, "lb", elb, tlb);	out << " ";
		print(out, "rb", erb, trb);	out << " ";
		print(out, "lt", elt, tlt); out << " ";
		print(out, "rt", ert, trt);	out << " ";
		print(out, "lst", elst, tlst); out << " ";
		print(out, "rst", erst, trst); out << " ";
		print(out, "back", eback, tback); out << " ";
		print(out, "start", estart, tstart); out << " ";
		print(out, "guide", eguide, tguide);
		out << endl;
	}

	void print(ostream & out, const char * btn, unsigned char e, int t){
		out << btn << ":"
			<< (EB_STDOWN & e ? "D" : "X") << (EB_EVDOWN & e ? "^" : "_")
			<< (EB_STUP & e ? "U" : "X") << (EB_EVUP & e ? "^" : "_") << " " << t;
	}

	// Event detectors.
	bool is_event_down(unsigned char e) const
	{
		return EB_EVDOWN & e ? true : false;
	}

	bool is_state_down(unsigned char e) const
	{
		return EB_STDOWN & e ? true : false;
	}

	bool is_event_up(unsigned char e) const
	{
		return EB_EVUP & e ? true : false;
	}

	bool is_state_up(unsigned char e) const
	{
		return EB_STUP & e ? true : false;
	}
};

#endif

