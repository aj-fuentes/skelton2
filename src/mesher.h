#ifndef MESHER_H
#define MESHER_H

#include "graph.h"
#include "scaffolder.h"
#include "field.h"

#include <string>
#include <map>
#include <iostream>
#include <sstream>


class Mesher;
typedef std::shared_ptr<Mesher> Mesher_ptr;
typedef std::vector<std::pair<Field_ptr,std::vector<int>>> PiecesParam;

class Mesher
{
public:
    Mesher(const Scaffolder_ptr& scaff, const Field_ptr& field, const PiecesParam& pieces, double lv) :
        scaff(scaff), field(field), pieces(pieces), lv(lv), max_quad_len(0.0), num_quads(1),
        num_quads_tip(1)
    {}
    std::vector<Point> compute_meshline(const Field_ptr&, const Point&, const UnitVector&, const Point&, const UnitVector&);
    std::vector<Point> compute_tip(const Field_ptr&, const Point&, const UnitVector&, const UnitVector&);
    void compute();
    void save_to_file(const std::string&) const;


    double max_quad_len;
    int num_quads;
    int num_quads_tip;

private:
    Scaffolder_ptr scaff;
    const Field_ptr field;
    PiecesParam pieces;
    double lv;


    std::vector<Point> points;
    std::map<Point,int> point_idxs;
    std::vector<std::tuple<int,int,int,int>> quads;
    std::vector<std::tuple<int,int,int>> tris;

};
#endif
