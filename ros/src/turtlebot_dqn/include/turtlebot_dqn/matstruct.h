#ifndef MATSTRUCT_H
#define MATSTRUCT_H

#include <matio.h>
#include <string>
#include <vector>
#include <memory>

namespace matio
{

typedef std::vector<char> Buffer;

/* Saves all information needed to construct a mat var */
class MatStructElement
{
public:
    MatStructElement(matio_classes matio_class, matio_types matio_type, const std::string& fieldname, size_t rows, size_t cols)
        : matio_class(matio_class), matio_type(matio_type), fieldname(fieldname), rows(rows), cols(cols) {}
    virtual ~MatStructElement() {}

    virtual void* get() = 0;

    matio_classes matio_class;
    matio_types matio_type;
    std::string fieldname;
    size_t rows, cols;
};

/* Specilization for scalar variables */
template< typename T >
class TypedMatStructElement : public MatStructElement
{
public:
    TypedMatStructElement(matio_classes matio_class, matio_types matio_type, const std::string& fieldname, const T& data, size_t rows = 1, size_t cols = 1)
        : MatStructElement(matio_class, matio_type, fieldname, rows, cols), data(data) {}

    /* pointer to data */
    void* get() { return (void*)(&data); }

private:
    T data;
};

/* Specilization for vector variables */
template< typename T >
class TypedVectorMatStructElement : public MatStructElement
{
public:
    TypedVectorMatStructElement(matio_classes matio_class, matio_types matio_type, const std::string& fieldname, const std::vector<T>& data, size_t rows = 1, size_t cols = 1)
        : MatStructElement(matio_class, matio_type, fieldname, rows, cols), data(data) {}

    /* pointer to first vector element */
    void* get() { return (void*)(&data[0]); }

private:
    std::vector<T> data;
};

/* constructs a mat struct */
class MatStruct
{
public:
    MatStruct();
    MatStruct(const std::string& filename, const std::string& structname);
    ~MatStruct();

    /* add new std::vector to mat struct */
    int add(const std::string& fieldname, const std::vector<double> &vec);
    int add(const std::string& fieldname, const std::vector<int>& vec);
    int add(const std::string& fieldname, const std::vector<float> &vec);

    /* add new number to mat struct */
    int add(const std::string& fieldname, double num);
    int add(const std::string& fieldname, int num );
    int add(const std::string& fieldname, float num);

    /* write data to disk */
    int write();
    int write(const std::string &filename, const std::string &structname);

private:
    matvar_t* matvar;
    std::string filename;
    std::string structname;
    std::vector< std::shared_ptr<MatStructElement> > elements;

    /* helper function to simply add new elements to a exsisting parent element */
    static void add_matvar(matvar_t* parent, int arr_idx, MatStructElement& elem);
};

}

#endif // MATSTRUCT_H
