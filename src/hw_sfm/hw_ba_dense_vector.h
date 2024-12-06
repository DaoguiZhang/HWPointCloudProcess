#ifndef HW_SFM_BA_DENSE_VECTOR_HEADER
#define HW_SFM_BA_DENSE_VECTOR_HEADER

#include<cmath>
#include<stdexcept>
#include<vector>

namespace HWSFM
{
	template <typename T>
	class HWDenseVector
	{
	public:
		HWDenseVector(void) = default;
		HWDenseVector(std::size_t size, T const& value = T(0));
		void resize(std::size_t size, T const& value = T(0));
		void clear(void);
		void fill(T const& value);
		std::size_t size(void) const;

		T* data(void);
		T const* data(void) const;
		T* begin(void);
		T const* begin(void) const;
		T* end(void);
		T const* end(void) const;

		HWDenseVector operator- (void) const;
		bool operator== (HWDenseVector const& rhs) const;
		T& operator[] (std::size_t index);
		T const& operator[] (std::size_t index) const;
		T& at(std::size_t index);
		T const& at(std::size_t index) const;

		T norm(void) const;
		T squared_norm(void) const;
		T dot(HWDenseVector const& rhs) const;
		HWDenseVector add(HWDenseVector const& rhs) const;
		HWDenseVector subtract(HWDenseVector const& rhs) const;
		HWDenseVector multiply(T const& factor) const;
		void multiply_self(T const& factor);
		void negate_self(void);

	private:
		std::vector<T> values;
	};

	/* ------------------------ Implementation ------------------------ */

	template <typename T>
	inline
		HWDenseVector<T>::HWDenseVector(std::size_t size, T const& value)
	{
		this->resize(size, value);
	}

	template <typename T>
	inline void
		HWDenseVector<T>::resize(std::size_t size, T const& value)
	{
		this->values.clear();
		this->values.resize(size, value);
	}

	template <typename T>
	inline void
		HWDenseVector<T>::clear(void)
	{
		this->values.clear();
	}

	template <typename T>
	inline void
		HWDenseVector<T>::fill(T const& value)
	{
		std::fill(this->values.begin(), this->values.end(), value);
	}

	template <typename T>
	std::size_t
		HWDenseVector<T>::size(void) const
	{
		return this->values.size();
	}

	template <typename T>
	T*
		HWDenseVector<T>::data(void)
	{
		return this->values.data();
	}

	template <typename T>
	T const*
		HWDenseVector<T>::data(void) const
	{
		return this->values.data();
	}

	template <typename T>
	T*
		HWDenseVector<T>::begin(void)
	{
		return this->values.data();
	}

	template <typename T>
	T const*
		HWDenseVector<T>::begin(void) const
	{
		return this->values.data();
	}

	template <typename T>
	T*
		HWDenseVector<T>::end(void)
	{
		return this->values.data() + this->values.size();
	}

	template <typename T>
	T const*
		HWDenseVector<T>::end(void) const
	{
		return this->values.data() + this->values.size();
	}

	template <typename T>
	HWDenseVector<T>
		HWDenseVector<T>::operator- (void) const
	{
		HWDenseVector ret(this->size());
		for (std::size_t i = 0; i < this->size(); ++i)
			ret[i] = -this->at(i);
		return ret;
	}

	template <typename T>
	bool
		HWDenseVector<T>::operator== (HWDenseVector const& rhs) const
	{
		if (this->size() != rhs.size())
			return false;
		for (std::size_t i = 0; i < this->size(); ++i)
			if (this->at(i) != rhs.at(i))
				return false;
		return true;
	}

	template <typename T>
	T&
		HWDenseVector<T>::operator[] (std::size_t index)
	{
		return this->values[index];
	}

	template <typename T>
	T const&
		HWDenseVector<T>::operator[] (std::size_t index) const
	{
		return this->values[index];
	}

	template <typename T>
	T&
		HWDenseVector<T>::at(std::size_t index)
	{
		return this->values[index];
	}

	template <typename T>
	T const&
		HWDenseVector<T>::at(std::size_t index) const
	{
		return this->values[index];
	}

	template <typename T>
	inline T
		HWDenseVector<T>::norm(void) const
	{
		return std::sqrt(this->squared_norm());
	}

	template <typename T>
	T
		HWDenseVector<T>::squared_norm(void) const
	{
		return this->dot(*this);
	}

	template <typename T>
	T
		HWDenseVector<T>::dot(HWDenseVector<T> const& rhs) const
	{
		if (this->size() != rhs.size())
			throw std::invalid_argument("Incompatible vector dimensions");

		T ret(0);
		for (std::size_t i = 0; i < this->size(); ++i)
			ret += this->values[i] * rhs.values[i];
		return ret;
	}

	template <typename T>
	HWDenseVector<T>
		HWDenseVector<T>::subtract(HWDenseVector<T> const& rhs) const
	{
		if (this->size() != rhs.size())
			throw std::invalid_argument("Incompatible vector dimensions");

		HWDenseVector<T> ret(this->size(), T(0));
		for (std::size_t i = 0; i < this->size(); ++i)
			ret.values[i] = this->values[i] - rhs.values[i];
		return ret;
	}

	template <typename T>
	HWDenseVector<T>
		HWDenseVector<T>::add(HWDenseVector<T> const& rhs) const
	{
		if (this->size() != rhs.size())
			throw std::invalid_argument("Incompatible vector dimensions");

		HWDenseVector<T> ret(this->size(), T(0));
		for (std::size_t i = 0; i < this->size(); ++i)
			ret.values[i] = this->values[i] + rhs.values[i];
		return ret;
	}

	template <typename T>
	HWDenseVector<T>
		HWDenseVector<T>::multiply(T const& factor) const
	{
		HWDenseVector<T> ret(this->size(), T(0));
		for (std::size_t i = 0; i < this->size(); ++i)
			ret[i] = this->at(i) * factor;
		return ret;
	}

	template <typename T>
	void
		HWDenseVector<T>::multiply_self(T const& factor)
	{
		for (std::size_t i = 0; i < this->size(); ++i)
			this->at(i) *= factor;
	}

	template <typename T>
	void
		HWDenseVector<T>::negate_self(void)
	{
		for (std::size_t i = 0; i < this->size(); ++i)
			this->at(i) = -this->at(i);
	}
}


#endif