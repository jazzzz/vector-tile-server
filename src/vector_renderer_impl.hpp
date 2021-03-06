/*****************************************************************************
 *
 * This file is part of Mapnik (c++ mapping toolkit)
 *
 * Copyright (C) 2012 Artem Pavlenko
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *****************************************************************************/

// mapnik
#include <mapnik/layer.hpp>
#include <mapnik/feature_type_style.hpp>
#include <mapnik/debug.hpp>
#include <mapnik/ctrans.hpp>
#include <mapnik/vertex_converters.hpp>

#include "vector_renderer.hpp"

// boost
#include <boost/utility.hpp>
#include <boost/make_shared.hpp>
#include "agg_basics.h"
#include "agg_path_storage.h"
#include "agg_conv_clipper.h"
namespace mapnik
{

template <typename T>
vector_renderer<T>::vector_renderer(Map const& m,
                                    T & backend,
                                    double scale_factor)
    : feature_style_processor<vector_renderer<T> >(m, scale_factor),
      backend_(backend),
      width_(m.width()),
      height_(m.height()),
      scale_factor_(scale_factor),
      t_(m.width(),m.height(),m.get_current_extent())
{
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer: Scale=" << m.scale();
}

template <typename T>
vector_renderer<T>::~vector_renderer() {}

template <typename T>
void vector_renderer<T>::start_map_processing(Map const& map)
{
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer: Start map processing bbox=" << map.get_current_extent();
}

template <typename T>
void vector_renderer<T>::end_map_processing(Map const& )
{
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer: End map processing";
}

template <typename T>
void vector_renderer<T>::start_layer_processing(layer const& lay, box2d<double> const& query_extent)
{
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer: Start processing layer=" << lay.name() ;
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer: -- datasource=" << lay.datasource().get();
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer: -- query_extent=" << query_extent;
    query_extent_ = query_extent;
}

template <typename T>
void vector_renderer<T>::end_layer_processing(layer const&)
{
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer: End layer processing";
}

template <typename T>
void vector_renderer<T>::start_style_processing(feature_type_style const& st)
{
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer:start style processing";
}

template <typename T>
void vector_renderer<T>::end_style_processing(feature_type_style const& st)
{
    MAPNIK_LOG_DEBUG(vector_renderer) << "vector_renderer:end style processing";
}

template <typename T>
void vector_renderer<T>::process(polygon_symbolizer const& sym,
                                 mapnik::feature_impl & feature,
                                 proj_transform const& prj_trans)
{
    agg::trans_affine tr;
    evaluate_transform(tr, feature, sym.get_transform());
    typedef boost::mpl::vector<transform_tag,affine_transform_tag,simplify_tag,smooth_tag> conv_types;
    agg::path_storage ps;

    double padding = (double) (query_extent_.width() / width_);
    padding *= 2;
    double x0 = query_extent_.minx();
    double y0 = query_extent_.miny();
    double x1 = query_extent_.maxx();
    double y1 = query_extent_.maxy();
    ps.move_to(x0 - padding, y0 - padding);
    ps.line_to(x0 - padding, y1 + padding);
    ps.line_to(x1 + padding, y1 + padding);
    ps.line_to(x1 + padding, y0 - padding);
    ps.close_polygon();

    vertex_converter<box2d<double>, backend_type, polygon_symbolizer,
                     CoordTransform, proj_transform, agg::trans_affine, conv_types>
    converter(query_extent_,backend_,sym,t_,prj_trans,tr,scale_factor_);

    converter.template set<transform_tag>(); //always transform
    converter.template set<affine_transform_tag>();
    if (sym.simplify_tolerance() > 0.0)
        converter.template set<simplify_tag>(); // optional simplify converter
    if (sym.smooth() > 0.0) converter.template set<smooth_tag>(); // optional smooth converter

    backend_.start_tile_element(feature, Polygon);

    BOOST_FOREACH( geometry_type & geom, feature.paths())
    {
        if (geom.size() > 2)
        {
            agg::conv_clipper<geometry_type, agg::path_storage> clp(geom, ps,
                    agg::clipper_and, agg::clipper_non_zero, agg::clipper_non_zero);
            // this seems to make it clockwise
            clp.reverse(true);
            converter.apply(clp);
        }
    }

    backend_.stop_tile_element();
}

template <typename T>
void vector_renderer<T>::process(line_symbolizer const& sym,
                                 mapnik::feature_impl & feature,
                                 proj_transform const& prj_trans)
{
    agg::trans_affine tr;
    evaluate_transform(tr, feature, sym.get_transform());

    box2d<double> clipping_extent = query_extent_;
    if (sym.clip())
    {
        double padding = (double)(query_extent_.width()/width_);
        padding *= 4;
        if (fabs(sym.offset()) > 0)
            padding *= fabs(sym.offset()) * 1.2;
        double x0 = query_extent_.minx();
        double y0 = query_extent_.miny();
        double x1 = query_extent_.maxx();
        double y1 = query_extent_.maxy();
        clipping_extent.init(x0 - padding, y0 - padding, x1 + padding , y1 + padding);
    }

    typedef boost::mpl::vector<clip_line_tag,transform_tag,affine_transform_tag,simplify_tag,smooth_tag> conv_types;
    vertex_converter<box2d<double>, backend_type, line_symbolizer,
                     CoordTransform, proj_transform, agg::trans_affine, conv_types>
        converter(clipping_extent,backend_,sym,t_,prj_trans,tr,scale_factor_);

    if (prj_trans.equal() && sym.clip()) converter.template set<clip_line_tag>(); //optional clip (default: true)
    converter.template set<transform_tag>(); //always transform
    converter.template set<affine_transform_tag>();
    if (sym.simplify_tolerance() > 0.0) converter.template set<simplify_tag>(); // optional simplify converter
    if (sym.smooth() > 0.0) converter.template set<smooth_tag>(); // optional smooth converter

    backend_.start_tile_element(feature,LineString);

    BOOST_FOREACH( geometry_type & geom, feature.paths())
    {
        if (geom.size() > 1)
        {
            converter.apply(geom);
        }
    }

    backend_.stop_tile_element();
}


template <typename T>
void vector_renderer<T>::process(point_symbolizer const& sym,
                                 mapnik::feature_impl & feature,
                                 proj_transform const& prj_trans)
{
    agg::trans_affine tr;
    evaluate_transform(tr, feature, sym.get_transform());
    typedef boost::mpl::vector<transform_tag,affine_transform_tag> conv_types;
    vertex_converter<box2d<double>, backend_type, point_symbolizer,
                     CoordTransform, proj_transform, agg::trans_affine, conv_types>
        converter(query_extent_,backend_,sym,t_,prj_trans,tr,scale_factor_);


    converter.template set<transform_tag>(); //always transform
    converter.template set<affine_transform_tag>();

    backend_.start_tile_element(feature,Point);

    BOOST_FOREACH( geometry_type & geom, feature.paths())
    {
        converter.apply(geom);
    }

    backend_.stop_tile_element();
}

}
