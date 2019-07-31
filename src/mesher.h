#ifndef MESHER_H
#define MESHER_H

#include "base.h"
#include "graph.h"
#include "scaffolder.h"
#include "field.h"

class Mesher;
typedef std::shared_ptr<Mesher> Mesher_ptr;
typedef std::vector<std::pair<Field_ptr,std::vector<int>>> PiecesParam;
typedef std::vector<Point> Meshline;

class Mesher
{
public:
    Mesher(const Scaffolder_ptr& scaff, const Field_ptr& field, const PiecesParam& pieces, double lv) :
        max_quad_len(0.0), num_quads(1), num_quads_tip(1), scaff(scaff), global_field(field),
        pieces(pieces), lv(lv)
    {}
    Meshline compute_meshline(const Field_ptr&, const Point&, const UnitVector&, const Point&, const UnitVector&);
    std::vector<Point> compute_tip(const Point&, const UnitVector&, const UnitVector&);
    void compute();
    void save_to_file(const std::string&, bool=false) const;


    double max_quad_len;
    int num_quads;
    int num_quads_tip;

private:
    Scaffolder_ptr scaff;
    const Field_ptr global_field;
    PiecesParam pieces;
    double lv;


    std::vector<Point> points;
    std::map<Point,int> point_idxs;
    std::vector<std::tuple<int,int,int,int>> quads;
    std::vector<std::tuple<int,int,int>> tris;

    std::vector<std::vector<std::pair<int,int>>> cells_match;

    void compute_cells_match();
    std::pair<const std::vector<Point>&,const std::vector<Point>&> get_cells(const std::vector<int> idxs) const;

    std::vector<std::vector<Meshline>> meshlines; //meshlines are associated to pieces
    std::vector<std::vector<Meshline>> tips;

};
#endif
