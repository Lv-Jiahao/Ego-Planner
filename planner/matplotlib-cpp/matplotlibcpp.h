#pragma once

#include <vector>
#include <map>
#include <array>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <cstdint>
#include <functional>

// Python.h must be included before any other header
#include <Python.h>

// Python 2/3 compatibility macros
#if PY_MAJOR_VERSION >= 3
#define PyString_FromString PyUnicode_FromString
#define PyInt_FromLong PyLong_FromLong
#define PyString_Check PyUnicode_Check
#endif

// Disable NumPy support to avoid constructor return value issues
#define WITHOUT_NUMPY

#ifndef WITHOUT_NUMPY
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#endif

namespace matplotlibcpp {

namespace detail {

static std::string s_backend;

struct _interpreter {
    PyObject *s_python_function_show;
    PyObject *s_python_function_close;
    PyObject *s_python_function_draw;
    PyObject *s_python_function_pause;
    PyObject *s_python_function_figure;
    PyObject *s_python_function_fignum_exists;
    PyObject *s_python_function_plot;
    PyObject *s_python_function_fill;
    PyObject *s_python_function_fill_between;
    PyObject *s_python_function_hist;
    PyObject *s_python_function_scatter;
    PyObject *s_python_function_subplot;
    PyObject *s_python_function_legend;
    PyObject *s_python_function_xlim;
    PyObject *s_python_function_ylim;
    PyObject *s_python_function_title;
    PyObject *s_python_function_axis;
    PyObject *s_python_function_xlabel;
    PyObject *s_python_function_ylabel;
    PyObject *s_python_function_grid;
    PyObject *s_python_function_clf;
    PyObject *s_python_function_errorbar;
    PyObject *s_python_function_annotate;
    PyObject *s_python_function_tight_layout;
    PyObject *s_python_function_stem;
    PyObject *s_python_function_xkcd;
    PyObject *s_python_function_text;
    PyObject *s_python_function_suptitle;
    PyObject *s_python_function_bar;
    PyObject *s_python_function_subplots_adjust;
    PyObject *s_python_function_savefig;
    PyObject *s_python_empty_tuple;
    PyObject *s_python_colorbar;

    _interpreter() {
        // Initialize Python
        Py_Initialize();

#ifndef WITHOUT_NUMPY
        import_array();
#endif

        PyObject* matplotlibname = PyString_FromString("matplotlib");
        PyObject* pyplotname = PyString_FromString("matplotlib.pyplot");
        PyObject* pylabname = PyString_FromString("pylab");
        
        if (!pyplotname || !pylabname || !matplotlibname) {
            throw std::runtime_error("couldnt create string");
        }

        PyObject* matplotlib = PyImport_Import(matplotlibname);
        Py_DECREF(matplotlibname);
        
        if (!matplotlib) {
            PyErr_Print();
            throw std::runtime_error("Error loading module matplotlib!");
        }

        // Set backend if specified
        if (!s_backend.empty()) {
            PyObject_CallMethod(matplotlib, const_cast<char*>("use"), const_cast<char*>("s"), s_backend.c_str());
        }

        PyObject* pymod = PyImport_Import(pyplotname);
        Py_DECREF(pyplotname);
        
        if (!pymod) {
            throw std::runtime_error("Error loading module matplotlib.pyplot!");
        }

        PyObject* pylabmod = PyImport_Import(pylabname);
        Py_DECREF(pylabname);
        
        if (!pylabmod) {
            throw std::runtime_error("Error loading module pylab!");
        }

        s_python_function_show = PyObject_GetAttrString(pymod, "show");
        s_python_function_close = PyObject_GetAttrString(pymod, "close");
        s_python_function_draw = PyObject_GetAttrString(pymod, "draw");
        s_python_function_pause = PyObject_GetAttrString(pymod, "pause");
        s_python_function_figure = PyObject_GetAttrString(pymod, "figure");
        s_python_function_fignum_exists = PyObject_GetAttrString(pymod, "fignum_exists");
        s_python_function_plot = PyObject_GetAttrString(pymod, "plot");
        s_python_function_fill = PyObject_GetAttrString(pymod, "fill");
        s_python_function_fill_between = PyObject_GetAttrString(pymod, "fill_between");
        s_python_function_hist = PyObject_GetAttrString(pymod, "hist");
        s_python_function_scatter = PyObject_GetAttrString(pymod, "scatter");
        s_python_function_subplot = PyObject_GetAttrString(pymod, "subplot");
        s_python_function_legend = PyObject_GetAttrString(pymod, "legend");
        s_python_function_ylim = PyObject_GetAttrString(pymod, "ylim");
        s_python_function_title = PyObject_GetAttrString(pymod, "title");
        s_python_function_axis = PyObject_GetAttrString(pymod, "axis");
        s_python_function_xlabel = PyObject_GetAttrString(pymod, "xlabel");
        s_python_function_ylabel = PyObject_GetAttrString(pymod, "ylabel");
        s_python_function_grid = PyObject_GetAttrString(pymod, "grid");
        s_python_function_xlim = PyObject_GetAttrString(pymod, "xlim");
        s_python_function_clf = PyObject_GetAttrString(pymod, "clf");
        s_python_function_errorbar = PyObject_GetAttrString(pymod, "errorbar");
        s_python_function_annotate = PyObject_GetAttrString(pymod, "annotate");
        s_python_function_tight_layout = PyObject_GetAttrString(pymod, "tight_layout");
        s_python_function_stem = PyObject_GetAttrString(pylabmod, "stem");
        s_python_function_xkcd = PyObject_GetAttrString(pymod, "xkcd");
        s_python_function_text = PyObject_GetAttrString(pymod, "text");
        s_python_function_suptitle = PyObject_GetAttrString(pymod, "suptitle");
        s_python_function_bar = PyObject_GetAttrString(pymod, "bar");
        s_python_function_subplots_adjust = PyObject_GetAttrString(pymod, "subplots_adjust");
        s_python_function_savefig = PyObject_GetAttrString(pymod, "savefig");
        s_python_colorbar = PyObject_GetAttrString(pymod, "colorbar");
        s_python_empty_tuple = PyTuple_New(0);
    }

    ~_interpreter() {
        Py_Finalize();
    }
};

static _interpreter& get_interpreter() {
    static _interpreter ctx;
    return ctx;
}

} // end namespace detail

// must be called before the first regular call to matplotlib to have any effect
void backend(const std::string& name) {
    detail::s_backend = name;
}

bool annotate(std::string annotation, double x, double y) {
    detail::_interpreter& s = detail::get_interpreter();

    PyObject* xy = PyTuple_New(2);
    PyObject* str = PyString_FromString(annotation.c_str());

    PyTuple_SetItem(xy, 0, PyFloat_FromDouble(x));
    PyTuple_SetItem(xy, 1, PyFloat_FromDouble(y));

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "xy", xy);

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, str);

    PyObject* res = PyObject_Call(s.s_python_function_annotate, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);

    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool plot(const std::vector<Numeric> &x, const std::vector<Numeric> &y, const std::string &s = "") {
    assert(x.size() == y.size());

    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* xarray = PyList_New(x.size());
    PyObject* yarray = PyList_New(y.size());

    for (size_t i = 0; i < x.size(); ++i) {
        PyList_SetItem(xarray, i, PyFloat_FromDouble(x.at(i)));
        PyList_SetItem(yarray, i, PyFloat_FromDouble(y.at(i)));
    }

    PyObject* pystring = PyString_FromString(s.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_plot, plot_args);

    Py_DECREF(plot_args);
    if (res) Py_DECREF(res);

    return res;
}

// Plot with keywords
template<typename Numeric>
bool plot(const std::vector<Numeric>& x, 
          const std::vector<Numeric>& y,
          const std::map<std::string, std::string>& keywords) {
    assert(x.size() == y.size());

    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* xarray = PyList_New(x.size());
    PyObject* yarray = PyList_New(y.size());

    for (size_t i = 0; i < x.size(); ++i) {
        PyList_SetItem(xarray, i, PyFloat_FromDouble(x.at(i)));
        PyList_SetItem(yarray, i, PyFloat_FromDouble(y.at(i)));
    }

    PyObject* plot_args = PyTuple_New(2);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);

    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(ctx.s_python_function_plot, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool scatter(const std::vector<Numeric>& x,
             const std::vector<Numeric>& y,
             const double s = 1.0) {
    assert(x.size() == y.size());

    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* xarray = PyList_New(x.size());
    PyObject* yarray = PyList_New(y.size());

    for (size_t i = 0; i < x.size(); ++i) {
        PyList_SetItem(xarray, i, PyFloat_FromDouble(x.at(i)));
        PyList_SetItem(yarray, i, PyFloat_FromDouble(y.at(i)));
    }

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "s", PyFloat_FromDouble(s));

    PyObject* plot_args = PyTuple_New(2);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);

    PyObject* res = PyObject_Call(ctx.s_python_function_scatter, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if (res) Py_DECREF(res);

    return res;
}

// Scatter with color mapping
template<typename Numeric>
bool scatter_colored(const std::vector<Numeric>& x,
                    const std::vector<Numeric>& y,
                    const std::vector<Numeric>& c,
                    const double s = 1.0,
                    const std::map<std::string, std::string>& keywords = {}) {
    assert(x.size() == y.size());
    assert(x.size() == c.size());

    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* xarray = PyList_New(x.size());
    PyObject* yarray = PyList_New(y.size());
    PyObject* carray = PyList_New(c.size());

    for (size_t i = 0; i < x.size(); ++i) {
        PyList_SetItem(xarray, i, PyFloat_FromDouble(x.at(i)));
        PyList_SetItem(yarray, i, PyFloat_FromDouble(y.at(i)));
        PyList_SetItem(carray, i, PyFloat_FromDouble(c.at(i)));
    }

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "c", carray);
    PyDict_SetItemString(kwargs, "s", PyFloat_FromDouble(s));
    
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* plot_args = PyTuple_New(2);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);

    PyObject* res = PyObject_Call(ctx.s_python_function_scatter, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool fill_between(const std::vector<Numeric>& x, const std::vector<Numeric>& y1, const std::vector<Numeric>& y2, const std::map<std::string, std::string>& keywords = {}) {
    assert(x.size() == y1.size());
    assert(x.size() == y2.size());

    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* xarray = PyList_New(x.size());
    PyObject* y1array = PyList_New(y1.size());
    PyObject* y2array = PyList_New(y2.size());

    for (size_t i = 0; i < x.size(); ++i) {
        PyList_SetItem(xarray, i, PyFloat_FromDouble(x[i]));
        PyList_SetItem(y1array, i, PyFloat_FromDouble(y1[i]));
        PyList_SetItem(y2array, i, PyFloat_FromDouble(y2[i]));
    }

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, y1array);
    PyTuple_SetItem(plot_args, 2, y2array);

    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(ctx.s_python_function_fill_between, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool hist(const std::vector<Numeric>& y, long bins = 10, std::string color = "b", double alpha = 1.0) {

    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* yarray = PyList_New(y.size());

    for (size_t i = 0; i < y.size(); ++i) {
        PyList_SetItem(yarray, i, PyFloat_FromDouble(y.at(i)));
    }

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "bins", PyLong_FromLong(bins));
    PyDict_SetItemString(kwargs, "color", PyString_FromString(color.c_str()));
    PyDict_SetItemString(kwargs, "alpha", PyFloat_FromDouble(alpha));

    PyObject* plot_args = PyTuple_New(1);

    PyTuple_SetItem(plot_args, 0, yarray);

    PyObject* res = PyObject_Call(ctx.s_python_function_hist, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if (res) Py_DECREF(res);

    return res;
}

inline bool subplot(long nrows, long ncols, long plot_number) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(nrows));
    PyTuple_SetItem(args, 1, PyFloat_FromDouble(ncols));
    PyTuple_SetItem(args, 2, PyFloat_FromDouble(plot_number));

    PyObject* res = PyObject_CallObject(ctx.s_python_function_subplot, args);

    Py_DECREF(args);
    if (res) Py_DECREF(res);

    return res;
}

inline bool title(const std::string &titlestr) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* pytitlestr = PyString_FromString(titlestr.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pytitlestr);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_title, args);
    if (!res) throw std::runtime_error("Call to title() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool axis(const std::string &axisstr) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* str = PyString_FromString(axisstr.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, str);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_axis, args);
    if (!res) throw std::runtime_error("Call to axis() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool xlabel(const std::string &str) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* pystr = PyString_FromString(str.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pystr);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_xlabel, args);
    if (!res) throw std::runtime_error("Call to xlabel() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool ylabel(const std::string &str) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* pystr = PyString_FromString(str.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pystr);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_ylabel, args);
    if (!res) throw std::runtime_error("Call to ylabel() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool grid(bool flag) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* pyflag = flag ? Py_True : Py_False;
    Py_INCREF(pyflag);

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pyflag);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_grid, args);
    if (!res) throw std::runtime_error("Call to grid() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool show(const bool block = true) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* res;
    if (block) {
        res = PyObject_CallObject(ctx.s_python_function_show, ctx.s_python_empty_tuple);
    } else {
        PyObject *kwargs = PyDict_New();
        PyDict_SetItemString(kwargs, "block", Py_False);
        res = PyObject_Call(ctx.s_python_function_show, ctx.s_python_empty_tuple, kwargs);
        Py_DECREF(kwargs);
    }

    if (!res) throw std::runtime_error("Call to show() failed.");

    Py_DECREF(res);

    return true;
}

inline bool close() {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* res = PyObject_CallObject(ctx.s_python_function_close, ctx.s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to close() failed.");

    Py_DECREF(res);

    return true;
}

inline bool draw() {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* res = PyObject_CallObject(ctx.s_python_function_draw, ctx.s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to draw() failed.");

    Py_DECREF(res);

    return true;
}

inline bool pause(double interval) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(interval));

    PyObject* res = PyObject_CallObject(ctx.s_python_function_pause, args);
    if (!res) throw std::runtime_error("Call to pause() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool save(const std::string& filename) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* pyfilename = PyString_FromString(filename.c_str());

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pyfilename);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_savefig, args);
    if (!res) throw std::runtime_error("Call to save() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool clf() {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* res = PyObject_CallObject(ctx.s_python_function_clf, ctx.s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to clf() failed.");

    Py_DECREF(res);

    return true;
}

inline bool ylim(double left, double right) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* list = PyList_New(2);
    PyList_SetItem(list, 0, PyFloat_FromDouble(left));
    PyList_SetItem(list, 1, PyFloat_FromDouble(right));

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, list);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_ylim, args);
    if (!res) throw std::runtime_error("Call to ylim() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool xlim(double left, double right) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* list = PyList_New(2);
    PyList_SetItem(list, 0, PyFloat_FromDouble(left));
    PyList_SetItem(list, 1, PyFloat_FromDouble(right));

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, list);

    PyObject* res = PyObject_CallObject(ctx.s_python_function_xlim, args);
    if (!res) throw std::runtime_error("Call to xlim() failed.");

    Py_DECREF(args);
    Py_DECREF(res);

    return true;
}

inline bool legend() {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* res = PyObject_CallObject(ctx.s_python_function_legend, ctx.s_python_empty_tuple);
    if (!res) throw std::runtime_error("Call to legend() failed.");

    Py_DECREF(res);

    return true;
}

inline bool tight_layout() {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* res = PyObject_CallObject(ctx.s_python_function_tight_layout, ctx.s_python_empty_tuple);
    if (!res) throw std::runtime_error("Call to tight_layout() failed.");

    Py_DECREF(res);

    return true;
}

inline long figure(long number = -1) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* args;
    if (number == -1)
        args = PyTuple_New(0);
    else {
        args = PyTuple_New(1);
        PyTuple_SetItem(args, 0, PyLong_FromLong(number));
    }

    PyObject* res = PyObject_CallObject(ctx.s_python_function_figure, args);
    if (!res) throw std::runtime_error("Call to figure() failed.");

    Py_DECREF(args);

    if (PyLong_Check(res)) {
        long ret = PyLong_AsLong(res);
        Py_DECREF(res);
        return ret;
    } else {
        Py_DECREF(res);
        return 0;
    }
}

inline bool fignum_exists(long number) {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, PyLong_FromLong(number));

    PyObject* res = PyObject_CallObject(ctx.s_python_function_fignum_exists, args);
    if (!res) throw std::runtime_error("Call to fignum_exists() failed.");

    bool ret = PyObject_IsTrue(res);

    Py_DECREF(args);
    Py_DECREF(res);

    return ret;
}

inline bool text(double x, double y, const std::string& s = "") {
    detail::_interpreter& ctx = detail::get_interpreter();

    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(x));
    PyTuple_SetItem(args, 1, PyFloat_FromDouble(y));
    PyTuple_SetItem(args, 2, PyString_FromString(s.c_str()));

    PyObject* res = PyObject_CallObject(ctx.s_python_function_text, args);

    Py_DECREF(args);
    if (res) Py_DECREF(res);

    return res;
}

} // end namespace matplotlibcpp