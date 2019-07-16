#include "mesher.h"

void Mesher::compute()
{


}

std::vector<Point> Mesher::compute_meshline(const Field_ptr& field, const Point& p, const UnitVector& u, const Point& q, const UnitVector& v)
{

    int num_quads = this->num_quads;
    if(max_quad_len>TOL)
        num_quads = std::max(int((p-q).norm()/max_quad_len),num_quads);


    //frames at extremities of the skeleton
    const Frame F0 = field->skel->get_frame(0);
    const Frame F1 = field->skel->get_frame(field->skel->l);

    const UnitVector w0 = F0.transpose()*u;
    const UnitVector w1 = F1.transpose()*v;

    std::vector<Point> points(num_quads+1);

    const double l = field->skel->l;
    for(int i=0;i<=num_quads;i++)
    {
        const double t = i*l/num_quads;
        const UnitVector wt = ((l-t)*w0 + t*w1).normalized();
        const double th = ((l-t)*field->th[0] + t*field->th[1])/l;
        const double cth = cos(th);
        const double sth = sin(th);
        const Frame F = field->skel->get_frame(t);
        Matrix R;
        R << 1,  0,   0,
             0,cth,-sth,
             0,sth, cth;
        const UnitVector w = F*R*wt;

        const Point pq = ((l-t)*p + t*q)/l;

        points[i] = this->field->shoot_ray(pq,w,lv);
    }

    return points;
}

std::vector<Point> Mesher::compute_tip(const Field_ptr&, const Point& p, const UnitVector& u,
    const UnitVector& v)
{

    int num_quads = this->num_quads_tip;
    std::vector<Point> points(num_quads+1);

    for(int i=0;i<=num_quads;i++)
    {
        const UnitVector w = (((num_quads-i)*u + i*v)/num_quads).normalized();
        points[i] = this->field->shoot_ray(p,w,lv);
    }

    return points;
}

void Mesher::save_to_file(const std::string& fname) const
{

}
