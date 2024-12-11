#ifndef BASIC_VIEW_H
#define BASIC_VIEW_H

#include <cairomm/context.h>
#include <cairomm/surface.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <pangomm/fontdescription.h>
#include <pangomm/layout.h>
#include <memory>


class View {
	
	public:
		
		View() = default;
		~View() = default;
		
		void initial(const int& _w, const int& _h,const std::string& name);
		void setLineWidth(const double& _l);
		//A=0 transparent, A=1 opaque
		void setRGBA(const double& _r, const double& _g, const double& _b, const double& _a);
		void setTranslate(const double& _x, const double& _y, const double & _l);
		void drawRectangle(const double& _x, const double& _y, const double& _w, const double& _h);
		void drawArc(const double&_x , const double&_y, const double& _r);
		void drawLine(const double&_sx , const double&_sy, const double& _ex, const double& _ey);
		void output();
	private:
		int width;
		int height;
		Cairo::RefPtr<Cairo::SvgSurface> surface;
		Cairo::RefPtr<Cairo::Context> p_context;

		double translatex;
		double translatey;
		double scale;

		double r;
		double g;
		double b;
		double a;
		double lw;

};
#endif
