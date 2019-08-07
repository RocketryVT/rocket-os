
#include <dynamic_matrix.hpp>

/// \brief The namespace used by all lambda classes and functions.
namespace lambda
{

/// \brief Copy constructor.
dynamic_matrix::dynamic_matrix(const dynamic_matrix &other)
    : _data(other.data()), _rows(other.rows()), _columns(other.columns()) { }

/// \brief Construct with dimensions only.
dynamic_matrix::dynamic_matrix(size_t rows, size_t columns)
    : _data(rows*columns), _rows(rows), _columns(columns) { }

/// \brief Construct with dimensions and contents.
dynamic_matrix::dynamic_matrix(size_t rows, size_t columns,
               const std::vector<double> &data)
    : _data(data), _rows(rows), _columns(columns)
{
    _data.resize(_rows*_columns);
}

/// \brief Assignment operator.
dynamic_matrix& dynamic_matrix::operator = (const dynamic_matrix &m)
{
    _data = m.data();
    _rows = m.rows();
    _columns = m.columns();
}

/// \brief Get the number of rows in this matrix.
size_t dynamic_matrix::rows() const
{
    return _rows;
}

/// \brief Get the number of columns in this matrix.
size_t dynamic_matrix::columns() const
{
    return _columns;
}

/// \brief Access an element in row i, column j.
double dynamic_matrix::operator () (size_t i, size_t j) const
{
    return _data[_columns*i + j];
}

/// \brief Get a reference to the element (i, j).
double& dynamic_matrix::operator () (size_t i, size_t j)
{
    return _data[_columns*i + j];
}

/// \brief Get the element at index i, interpreting the matrix
/// as a 1-dimensional array in a row-major fashion.
double dynamic_matrix::operator [] (size_t i) const
{
    return _data[i];
}

/// \brief Get a reference to the element at i, in the
/// equivalent 1-dimensional row-major array.
double& dynamic_matrix::operator [] (size_t i)
{
    return _data[i];
}

/// \brief Same as operator [], but throws an exception if
/// the index provided is out of bounds.
double dynamic_matrix::at(size_t i) const
{
    range_check(i);
    return _data[i];
}

/// \brief Same as operator [], but throws an exception if
/// the index provided is out of bounds.
double& dynamic_matrix::at(size_t i)
{
    range_check(i);
    return _data[i];
}

/// \brief Same as operator (), but throws an exception if
/// the index provided is out of bounds.
double dynamic_matrix::at(size_t i, size_t j) const
{
    range_check(i, j);
    return _data[_columns*i + j];
}

/// \brief Same as operator (), but throws an exception if
/// the index provided is out of bounds.
double& dynamic_matrix::at(size_t i, size_t j)
{
    range_check(i, j);
    return _data[_columns*i + j];
}

/// \brief Get a const ref to the underlying data of this matrix.
const std::vector<double>& dynamic_matrix::data() const
{
    return _data;
}

/// \brief Throws an exception if an index is out of bounds.
void dynamic_matrix::range_check(size_t i) const
{
    if (i >= _data.size())
    {
        std::stringstream ss;
        ss << "Cannot access element (" << i
            << ") of " << _rows << "x" << _columns << " matrix";
        throw std::out_of_range(ss.str());
    }
}

/// \brief Throws an exception if an index is out of bounds.
void dynamic_matrix::range_check(size_t i, size_t j) const
{
    if (i >= _rows || j >= _columns || i < 0 || j < 0)
    {
        std::stringstream ss;
        ss << "Cannot access element (" << i << ", " << j
            << ") of " << _rows << "x" << _columns << " matrix";
        throw std::out_of_range(ss.str());
    }
}

bool operator == (const dynamic_matrix &left,
                  const dynamic_matrix &right)
{
    if (left.rows() != right.rows() ||
        left.columns() != right.columns())
    {
        return false;
    }
    for (size_t i = 0; i < left.rows()*left.columns(); ++i)
    {
        if (left[i] != right[i]) return false;
    }
    return true;
}

/// \brief Print a matrix to a std::ostream.
std::ostream& operator << (std::ostream &os, const dynamic_matrix &m)
{
    const size_t M = m.rows(), N = m.columns();
    os.precision(3);
    os.setf(std::ios::fixed);
    os << "[";
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            os << +m(i, j);
            if (j < N - 1) os << ", ";
        }
        if (i < M - 1) os << "; ";
    }
    os << "]";
}

/// \brief Produces a multiline string representation of a matrix.
std::string pretty(const dynamic_matrix &m)
{
    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    const size_t M = m.rows(), N = m.columns();

    size_t max_len = 0;
    for (size_t i = 0; i < M*N; ++i)
    {
        std::stringstream tiny;
        tiny.precision(3);
        tiny.setf(std::ios::fixed);
        tiny << m[i];
        size_t len = tiny.tellp();
        if (len > max_len) max_len = len;
    }

    for (size_t r = 0; r < M; ++r)
    {
        ss << "  [";
        for (size_t c = 0; c < N; ++c)
        {
            ss << std::setw(max_len + 1) << m(r, c);
        }
        ss << " ]\n";
    }

    return ss.str();
}

/// \brief Addition of two matrices.
dynamic_matrix operator + (const dynamic_matrix &left,
                           const dynamic_matrix &right)
{
    if (left.columns() != right.columns() ||
        left.rows() != right.rows())
    {
        std::stringstream ss;
        ss << "Cannot add matrices of dimensions "
            << left.rows() << "x" << left.columns() << " and "
            << right.rows() << "x" << right.columns();
        throw std::invalid_argument(ss.str());
    }

    const size_t M = left.rows(), N = left.columns();
    dynamic_matrix ret(M, N);
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            ret(i, j) = left(i, j) + right(i, j);
        }
    }
    return ret;
}

/// \brief Multiplication of two matrices.
dynamic_matrix operator * (const dynamic_matrix &left,
                           const dynamic_matrix &right)
{
    if (left.columns() != right.rows())
    {
        std::stringstream ss;
        ss << "Cannot multiply matrices of dimensions "
            << left.rows() << "x" << left.columns() << " and "
            << right.rows() << "x" << right.columns();
        throw std::invalid_argument(ss.str());
    }

    const size_t M = left.rows(), N = left.columns(), P = right.columns();
    dynamic_matrix ret(M, P);
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < P; ++j)
        {
            double sum = 0;
            for (size_t k = 0; k < N; ++k)
            {
                sum += left(i, k) * right(k, j);
            }
            ret(i, j) = sum;
        }
    }
    return ret;
}

} // namespace lambda
