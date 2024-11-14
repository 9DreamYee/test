#include "View.h"
void View::initial(const int& _w, const int& _h,const std::string& name){
	width  = _w;
	height = _h;
	std::string svgName{"picture/"};
	svgName+= name;
	svgName+=".svg";
	surface = Cairo::SvgSurface::create(svgName, _w, _h);
	p_context = Cairo::Context::create(surface);
	//binack ground is ground
	p_context->set_source_rgba(0.8,0.8,0.8,0.2);
	p_context->paint();

}
void View::setTranslate(const double& _x, const double& _y, const double& _l){
	//input parameter is the smallest x the biggest y in the design
	//first set translate space to window's spaceace
	//calculate scale
	scale = (height>width) ? width/_l : height/_l;
	translatex = -1.0*_x*scale;
	translatey = -1.0*-1.0*_y*scale; //first is flip
	if(width> height){
		translatex += (double)(width-height) / (2.0);
	}
	else if((width -height) != 0){
		translatey += (double)(height-width) / (2.0);
	}

	//std::cout<<translatex<<","<<translatey<<std::endl;
}
void View::setLineWidth(const double& _l){
	lw = _l;
}
void View::setRGBA(const double& _r, const double& _g, const double& _b, const double& _a){
	r = _r;
	g = _g;
	b = _b;
	a = _a;
}
void View::drawRectangle(const double& _x, const double& _y, const double& _w, const double& _h){
	p_context->set_line_width(lw);
	p_context->set_source_rgba(r,g,b,a);
	double tx = _x*scale;
	double ty = _y*(-1.0)*scale;
	p_context->rectangle(tx+translatex,ty+translatey,_w*scale,_h*scale);
	p_context->stroke();
	p_context->save();
}
void View::drawArc(const double&_x , const double&_y, const double& _r){
	p_context->set_line_width(lw);
	p_context->set_source_rgba(r,g,b,a);
	//std::cout<<_x<<","<<_y<<"("<<translatex<<","<<translatey<<")->        ";
	double tx = _x*scale;
	double ty = _y*(-1.0)*scale;
	//std::cout<<tx<<","<<ty<<"->        ";
	//std::cout<<tx+translatex<<","<<ty+translatey<<std::endl;
	p_context->arc(tx+translatex,ty+translatey,_r*scale,0,2*M_PI);
	p_context->fill();
	p_context->save();
}
void View::drawLine(const double&_sx , const double&_sy, const double& _ex, const double& _ey){
	p_context->set_line_width(lw);
	p_context->set_source_rgba(r,g,b,a);
	double tsx = _sx*scale;
	double tsy = _sy*(-1.0)*scale;
	double tex = _ex*scale;
	double tey = _ey*(-1.0)*scale;
	p_context->move_to(tsx+translatex,tsy+translatey);
	p_context->line_to(tex+translatex,tey+translatey);
	p_context->stroke();
 	p_context->save();
}
void View::output(){
	p_context->show_page();
	surface->finish();
}
