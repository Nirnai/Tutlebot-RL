#include "turtlebot_dqn/matstruct.h"

using namespace matio;

MatStruct::MatStruct()
{
}

MatStruct::MatStruct(const std::string& filename, const std::string& structname)
    : filename(filename), structname(structname)
{
}

MatStruct::~MatStruct()
{
    Mat_VarFree(matvar);
}

int MatStruct::write(const std::string &filename, const std::string &structname)
{
    this->filename = filename;
    this->structname = structname;
    return write();
}

int MatStruct::write()
{
    // 1. open new mat file
    mat_t *matfp;
    matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT5);
    if(matfp == NULL) {
        return -1; // error creating file
    }

    // 2. create fieldname vector
    std::vector<const char*> fieldnames;
    fieldnames.reserve(elements.size());
    for(size_t i = 0; i < elements.size(); ++i)
       fieldnames.push_back(const_cast<char*>( elements[i]->fieldname.c_str() ) );

    // 3. initalize matvar to hold struct
    char nfields = elements.size();
    size_t struct_dims[2] = {1, 1};
    matvar = Mat_VarCreateStruct(structname.c_str(), 2, struct_dims, &(fieldnames[0]), nfields);

    // 4. add elements to struct matvar
    for(size_t i = 0; i < elements.size(); ++i) {
        add_matvar(matvar, 0, *elements[i] );
    }

    // 5. write to file
    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
    Mat_Close(matfp);
    return 0;
}

int MatStruct::add(const std::string& fieldname, const std::vector<double>& vec)
{
    elements.push_back( std::shared_ptr<MatStructElement>( new TypedVectorMatStructElement< double >(MAT_C_DOUBLE, MAT_T_DOUBLE, fieldname, vec, vec.size() ) ) );
    return 0;
}

int MatStruct::add(const std::string& fieldname, const std::vector<int>& vec)
{
    elements.push_back( std::shared_ptr<MatStructElement>( new TypedVectorMatStructElement< int >(MAT_C_INT32, MAT_T_INT32, fieldname, vec, vec.size() ) ) );
    return 0;
}

int MatStruct::add(const std::string& fieldname, const std::vector<float>& vec)
{
    elements.push_back( std::shared_ptr<MatStructElement>( new TypedVectorMatStructElement< float >(MAT_C_SINGLE, MAT_T_SINGLE, fieldname, vec, vec.size() ) ) );
    return 0;
}

int MatStruct::add(const std::string& fieldname, double num)
{
    elements.push_back( std::shared_ptr<MatStructElement>( new TypedMatStructElement< double >(MAT_C_DOUBLE, MAT_T_DOUBLE, fieldname, num ) ) );
    return 0;
}

int MatStruct::add(const std::string& fieldname, int num )
{
    elements.push_back( std::shared_ptr<MatStructElement>( new TypedMatStructElement< int >(MAT_C_INT32, MAT_T_INT32, fieldname, num ) ) );
    return 0;
}

int MatStruct::add(const std::string& fieldname, float num)
{
    elements.push_back( std::shared_ptr<MatStructElement>( new TypedMatStructElement< float >(MAT_C_SINGLE, MAT_T_SINGLE, fieldname, num ) ) );
    return 0;
}

void MatStruct::add_matvar(matvar_t* parent, int arr_idx, MatStructElement& elem)
{
    size_t var_dims[2];
    var_dims[0] = elem.rows;
    var_dims[1] = elem.cols;
    matvar_t* child = Mat_VarCreate(NULL, elem.matio_class, elem.matio_type, 2, var_dims, elem.get(), MAT_F_DONT_COPY_DATA);
    Mat_VarSetStructFieldByName(parent, elem.fieldname.c_str(), arr_idx, child);
}
