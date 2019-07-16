#include "mesher.h"

std::pair<const std::vector<Point>&,const std::vector<Point>&> Mesher::get_cells(const std::vector<int> idxs) const
{
    const auto& i = idxs[0];
    const Edge e_i = Edge(i,idxs[1]);

    const auto& j = idxs[idxs.size()-1];
    const Edge e_j = Edge(j,idxs[idxs.size()-2]);

    return {scaff->cells.at({i,e_i}),scaff->cells.at({j,e_j})};
}

void Mesher::compute_cells_match()
{
    const auto& g = scaff->g;
    const auto& cells = scaff->cells;

    for(const auto& piece : pieces)
    {
        const auto& field = piece.first;

        const Frame F1t = field->skel->get_frame(0).transpose();
        const Frame F2t = field->skel->get_frame(field->skel->l).transpose();

        const auto& cells = get_cells(piece.second);

        const auto& cell_points1 = cells.first;
        const auto& cell_points2 = cells.second;

        const int n = cell_points1.size();

        std::vector<Point> points1(n);
        std::vector<Point> points2(n);

        for(int i=0;i<n;i++)
        {
            points1[i] = F1t*cell_points1[i];
            points2[i] = F2t*cell_points2[i];
        }

        cells_match.push_back(Scaffolder::math_cells(points1,points2));
    }
}

void Mesher::compute()
{
    compute_cells_match();

    const auto& g = scaff->g;
    const auto& nodes = g->get_nodes();

    meshlines = std::vector<std::vector<Meshline>>(pieces.size());

    for(int i=0;i<pieces.size();i++)
    {
        const auto& piece = pieces[i];

        const auto& field = piece.first;
        const auto& idxs = piece.second;

        const auto& cells = get_cells(idxs);

        const auto& p = nodes[idxs[0]];
        const auto& q = nodes[idxs[idxs.size()-1]];

        for(const auto& match : cells_match[i])
        {
            const auto& u = cells.first[match.first];
            const auto& v = cells.second[match.second];
            meshlines[i].push_back(compute_meshline(field,p,u,q,v));
        }
    }

    const auto& scaff_cells = scaff->cells;
    for(int i=0;i<nodes.size();i++)
    {
        if(g->is_dangling(i))
        {
            const auto& p = nodes[i];
            const Edge e = g->get_incident_edges(i)[0];
            const UnitVector v = (nodes[e.i!=i? e.i : e.j]-p).normalized();
            const auto& cell = scaff_cells.at({i,e});
            for(const auto& u : cell)
                tips.push_back(compute_tip(p,u,v));
        }
    }
}

Meshline Mesher::compute_meshline(const Field_ptr& field, const Point& p, const UnitVector& u, const Point& q, const UnitVector& v)
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

std::vector<Point> Mesher::compute_tip(const Point& p, const UnitVector& u, const UnitVector& v)
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
