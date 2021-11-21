#include <iostream>
#include <vector>
#include <random>
#include <array>
#include <mutex>
#include <thread>
#include <chrono>
#include <time.h>


#ifdef WIN32
using pointer_t = uint32_t;
#else
using pointer_t = uint64_t;
#endif

//#define ALLOC_FREE_CHECK

class Test01
{
public:
	Test01()
		: ary01()
	{
#ifdef ALLOC_FREE_CHECK
		std::cout << "Test01" << std::endl;
#endif
	}
	~Test01()
	{
#ifdef ALLOC_FREE_CHECK
		std::cout << "~Test01" << std::endl;
#endif
	}
private:
	long ary01[3];
};
static_assert(sizeof(Test01) == 12, "");

class Test02
{
	long ary01[11];
};
static_assert(sizeof(Test02) == 44, "");

class Test03
{
	long ary01[28];
};
static_assert(sizeof(Test03) == 112, "");

class Test04
{
	long ary01[55];
};
static_assert(sizeof(Test04) == 220, "");

class Test05
{
	long ary01[105];
};
static_assert(sizeof(Test05) == 420, "");




class Test11
{
	long ary01[3];
public:
	virtual ~Test11() {};
};
static_assert(sizeof(Test11) == 24, "");

class Test12 : public Test11
{
	long ary01[11];
public:
	virtual ~Test12() override {};
};
static_assert(sizeof(Test12) == 72, "");

class Test13 : public Test12
{
	long ary01[28];
public:
	virtual ~Test13() override {};
};
static_assert(sizeof(Test13) == 184, "");

class Test14 : public Test13
{
	long ary01[55];
public:
	virtual ~Test14() override {};
};
static_assert(sizeof(Test14) == 408, "");

class Test15 : public Test14
{
	long ary01[105];
public:
	virtual ~Test15() override {};
};
static_assert(sizeof(Test15) == 832, "");


class MemoryPool
{
private:
	struct BlockHeader final
	{
		uint32_t refIdx : 11;
		uint32_t size : 21;
	};
	struct BlockHeaderRef final
	{
		BlockHeaderRef()
			: headerAddress(0)
			, mutex()
		{
		}
		pointer_t headerAddress;
		std::mutex mutex;
	};

	class AllBlockScopedLock final
	{
	public:
		AllBlockScopedLock(MemoryPool& pool)
			: m_pool(pool)
		{
			//頭からロック取っていくので、頭のロックを取れるかどうかでどちらの命令が全てのロックを取るかが決まる
			//デッドロックは起こらないはず
			uint8_t* pCurrent = m_pool.m_pBuff[BUFF_TYPE_HeaderRef];
			uint8_t* pEnd = m_pool.m_pBuff[BUFF_TYPE_HeaderRef] + HEADER_REF_SIZE;
			while (pEnd > pCurrent)
			{
				BlockHeaderRef* pHederRef = reinterpret_cast<BlockHeaderRef*>(pCurrent);
				pHederRef->mutex.lock();
				pCurrent += sizeof(BlockHeaderRef);
			}
		}
		~AllBlockScopedLock()
		{
			//アンロックも頭からすればその分無駄が無いはず
			uint8_t* pCurrent = m_pool.m_pBuff[BUFF_TYPE_HeaderRef];
			uint8_t* pEnd = m_pool.m_pBuff[BUFF_TYPE_HeaderRef] + HEADER_REF_SIZE;
			while (pEnd > pCurrent)
			{
				BlockHeaderRef* pHederRef = reinterpret_cast<BlockHeaderRef*>(pCurrent);
				pHederRef->mutex.unlock();
				pCurrent += sizeof(BlockHeaderRef);
			}
		}

	private:
		MemoryPool& m_pool;
	};

public:
	template<typename Type>
	class Handle
	{
	public:
		Handle()
			: m_pPool(nullptr)
			, m_pHeaderRef(nullptr)
		{
		}
		Handle(MemoryPool* pPool, BlockHeaderRef* pHeaderRef)
			: m_pPool(pPool)
			, m_pHeaderRef(pHeaderRef)
		{
		}
		Handle(Handle&& rhs) noexcept
			: m_pPool(rhs.m_pPool)
			, m_pHeaderRef(rhs.m_pHeaderRef)
		{
			rhs.m_pPool = nullptr;
			rhs.m_pHeaderRef = nullptr;
		}
		~Handle()
		{
			Release();
		}

		Handle& operator=(Handle&& rhs) noexcept
		{
			m_pPool = rhs.m_pPool;
			m_pHeaderRef = rhs.m_pHeaderRef;

			rhs.m_pPool = nullptr;
			rhs.m_pHeaderRef = nullptr;

			return *this;
		}

	public:
		Handle(const Handle&) = delete;
		Handle& operator=(const Handle&) = delete;

	public:
		bool IsValid() const { return nullptr != m_pHeaderRef; }
		bool IsInvalid() const { return !IsValid(); }

		void Release()
		{
			if (IsValid())
			{
				m_pPool->_ReleaseBlock<Type>(m_pHeaderRef);

				//アクセス出来ないようにnullにする
				m_pHeaderRef = nullptr;
			}
		}

	private:
		MemoryPool* m_pPool;
		BlockHeaderRef* m_pHeaderRef;
	};

	template<typename Type>
	class ProxyObj
	{
	public:
		ProxyObj(MemoryPool* pPool, BlockHeaderRef* pHeaderRef)
			: m_pPool(pPool)
			, m_pHeaderRef(pHeaderRef)
		{
			m_pHeaderRef->mutex.lock();
		}
		~ProxyObj()
		{
			m_pHeaderRef->mutex.unlock();
		}

	private:
		ProxyObj(const ProxyObj<Type>&) = delete;
		ProxyObj<Type>& operator=(const ProxyObj<Type>&) = delete;

	public:
		operator Type&()
		{
			return *(m_pPool->_GetLockObj<Type>(m_pHeaderRef));
		}
		operator const Type&() const
		{
			return *(m_pPool->_GetLockObj<Type>(m_pHeaderRef));
		}

	private:
		MemoryPool* m_pPool;
		BlockHeaderRef* m_pHeaderRef;
	};

public:
	MemoryPool()
		: m_pBuff{}

		, m_dbgIsUseHeap(false)
	{
		m_pBuff[BUFF_TYPE_Pool_00] = reinterpret_cast<uint8_t*>(std::calloc(POOL_SIZE_00, sizeof(uint8_t)));
		m_pBuff[BUFF_TYPE_HeaderRef] = reinterpret_cast<uint8_t*>(new BlockHeaderRef[HEADER_REF_NUM]);

		//ヘッダも埋め込んでおく
		BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(m_pBuff[BUFF_TYPE_Pool_00]);
		pBlockHeader->refIdx = HEADER_REF_NUM;
		pBlockHeader->size = POOL_SIZE_00 - sizeof(BlockHeader);
	}
	~MemoryPool()
	{
		delete[] reinterpret_cast<BlockHeaderRef*>(m_pBuff[BUFF_TYPE_HeaderRef]);
		std::free(m_pBuff[BUFF_TYPE_Pool_00]);
	}

public:
	template<typename Type, class... Args>
	Handle<Type> CreateObject(Args... args)
	{
		AllBlockScopedLock lock(*this);

		//生成
		BlockHeaderRef* pUseableHeaderRef = _SearchUsealeHeaderRef();
		if (nullptr != pUseableHeaderRef)
		{
			BlockHeader* pUseableHeader = _SearchUseableHeader(sizeof(Type));
			if (nullptr != pUseableHeader)//プール使用
			{
				//ヘッダ情報書き換え
				uint8_t* pU8BlockHeader = reinterpret_cast<uint8_t*>(pUseableHeader);
				pUseableHeader->refIdx = pUseableHeaderRef - reinterpret_cast<BlockHeaderRef*>(m_pBuff[BUFF_TYPE_HeaderRef]);

				if (sizeof(Type) > pUseableHeader->size)
				{
					_ASSERT_EXPR(false, "使用可能領域としてサイズの足りないメモリブロックを戻した");
				}
				else if (sizeof(Type) + sizeof(BlockHeader) > pUseableHeader->size)
				{
					//次の未使用領域にヘッダを埋め込むサイズも無いならまとめてしまう
				}
				else
				{
					//次のヘッダ情報を書き換え
					BlockHeader* pNextBlockHeader = reinterpret_cast<BlockHeader*>(pU8BlockHeader + (sizeof(BlockHeader) + sizeof(Type)));
					pNextBlockHeader->refIdx = HEADER_REF_NUM;
					pNextBlockHeader->size = pUseableHeader->size - (sizeof(Type) + sizeof(BlockHeader));

					pUseableHeader->size = sizeof(Type);
				}

				//オブジェクト生成
				Type* pObject = new(pU8BlockHeader + sizeof(BlockHeader)) Type(args...);

				//ヘッダ参照
				pUseableHeaderRef->headerAddress = reinterpret_cast<pointer_t>(pUseableHeader);
			}
			else//ヒープ使用
			{
				//オブジェクト生成
				Type* pObject = new Type(args...);

				//ヘッダ参照
				pUseableHeaderRef->headerAddress = reinterpret_cast<pointer_t>(pObject);


				m_dbgIsUseHeap = true;
			}
		}
		else
		{
			_ASSERT_EXPR(false, "最大ブロックサイズが足りない");
		}

		return Handle<Type>(this, pUseableHeaderRef);
	}

	void Compaction(int num)
	{
		AllBlockScopedLock lock(*this);

		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_Pool_00];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		BlockHeader* pUseableHeader = nullptr;
		size_t useableSize = 0;

		int currentNum = 0;
		while (pEnd > pCurrent)
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			if (HEADER_REF_NUM == pBlockHeader->refIdx)
			{
				if (nullptr == pUseableHeader)
				{
					pUseableHeader = pBlockHeader;
					useableSize = pBlockHeader->size;
				}
				else
				{
					useableSize += sizeof(BlockHeader) + pBlockHeader->size;
				}
			}
			else if(nullptr != pUseableHeader)
			{
				//未使用メモリ使って複製
				pUseableHeader->refIdx = pBlockHeader->refIdx;
				pUseableHeader->size = pBlockHeader->size;
				std::memmove(
					reinterpret_cast<uint8_t*>(pUseableHeader) + sizeof(BlockHeader),
					reinterpret_cast<uint8_t*>(pBlockHeader) + sizeof(BlockHeader),
					pBlockHeader->size);

				//コンパクションした次のメモリブロック
				BlockHeader* pNextBlockHeader =
					reinterpret_cast<BlockHeader*>(reinterpret_cast<uint8_t*>(pUseableHeader) + sizeof(BlockHeader) + pUseableHeader->size);
				pNextBlockHeader->refIdx = HEADER_REF_NUM;
				pNextBlockHeader->size = useableSize;

				//ヘッダの参照を差し替え
				BlockHeaderRef* pHeaderRef = reinterpret_cast<BlockHeaderRef*>(m_pBuff[BUFF_TYPE_HeaderRef]) + pUseableHeader->refIdx;
				pHeaderRef->headerAddress = reinterpret_cast<pointer_t>(pUseableHeader);

				pBlockHeader = pUseableHeader;
				pCurrent = reinterpret_cast<uint8_t*>(pUseableHeader);

				++currentNum;

				if (num > currentNum)
				{
					pUseableHeader = nullptr;
				}
				else
				{
					break;
				}
			}

			pCurrent += sizeof(BlockHeader) + pBlockHeader->size;
		}
	}

private:
	bool _IsPoolRange(const uint8_t* pCurrent) const
	{
		return
			(m_pBuff[BUFF_TYPE_Pool_00] <= pCurrent) &&
			(m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00 > pCurrent);
	}

	BlockHeader* _SearchUseableHeader(size_t needsByte)
	{
		uint8_t* pCurrent = reinterpret_cast<uint8_t*>(m_pBuff[BUFF_TYPE_Pool_00]);
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		BlockHeader* pUseableHeader = nullptr;
		size_t useableSize = 0;

		while (pEnd > pCurrent)
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			if (HEADER_REF_NUM == pBlockHeader->refIdx)
			{
				if (0 == useableSize)//連続未使用ブロックの先頭
				{
					pUseableHeader = pBlockHeader;
					useableSize = pBlockHeader->size;
				}
				else//連続未使用ブロックの先頭以外
				{
					useableSize += sizeof(BlockHeader) + pBlockHeader->size;
				}

				if (needsByte <= useableSize)
				{
					//必要量以上に使用可能メモリを見つけたらヘッダを書き換えて戻す
					pUseableHeader->size = useableSize;
					return pUseableHeader;
				}
			}
			else
			{
				useableSize = 0;
			}

			pCurrent += sizeof(BlockHeader) + pBlockHeader->size;
		}

		return nullptr;
	}

	BlockHeaderRef* _SearchUsealeHeaderRef()
	{
		uint8_t* pCurrent = reinterpret_cast<uint8_t*>(m_pBuff[BUFF_TYPE_HeaderRef]);
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_HeaderRef] + HEADER_REF_SIZE;
		while (pEnd > pCurrent)
		{
			BlockHeaderRef* pHederRef = reinterpret_cast<BlockHeaderRef*>(pCurrent);

			//未使用メモリならヘッダの参照を戻す
			if (0 == pHederRef->headerAddress)
			{
				return pHederRef;
			}

			pCurrent += sizeof(BlockHeaderRef);
		}

		return nullptr;
	}

	template<typename Type>
	void _ReleaseBlock(BlockHeaderRef* pHeaderRef)
	{
		std::lock_guard<std::mutex> lock(pHeaderRef->mutex);

		//破棄
		uint8_t* pU8BlockHeader = reinterpret_cast<uint8_t*>(pHeaderRef->headerAddress);
		if (_IsPoolRange(pU8BlockHeader))
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pU8BlockHeader);
			if (0 < pBlockHeader->size)
			{
				Type* pObject = reinterpret_cast<Type*>(pU8BlockHeader + sizeof(BlockHeader));
				pObject->~Type();

				//ヘッダ情報の書き換え
				pBlockHeader->refIdx = HEADER_REF_NUM;
			}
			else
			{
				_ASSERT_EXPR(false, "サイズが0なオブジェクトを破棄");
			}
		}
		else
		{
			Type* pObject = reinterpret_cast<Type*>(pU8BlockHeader);
			delete pObject;
		}

		//ヘッダ参照情報をクリアする
		pHeaderRef->headerAddress = 0;
	}

	template<typename Type>
	Type* _GetLockObj(BlockHeaderRef* pHeaderRef) const
	{
		pHeaderRef->mutex.lock();

		//破棄
		uint8_t* pU8BlockHeader = reinterpret_cast<uint8_t*>(pHeaderRef->headerAddress);
		if (_IsPoolRange(pU8BlockHeader))
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pU8BlockHeader);
			Type* pObject = reinterpret_cast<Type*>(pU8BlockHeader + sizeof(BlockHeader));
			return pObject;
		}
		else
		{
			return reinterpret_cast<Type*>(pU8BlockHeader);
		}
	}

private:
	static constexpr size_t POOL_SIZE_00 = 1024 * 300;//プール０の最大バイト数
	static constexpr int HEADER_REF_NUM = 1200;
	static constexpr size_t HEADER_REF_SIZE = sizeof(BlockHeaderRef) * HEADER_REF_NUM;//ヘッダ参照領域の最大バイト数

	enum BUFF_TYPE : int
	{
		BUFF_TYPE_Pool_00, 
		BUFF_TYPE_HeaderRef, 

		BUFF_TYPE_Num
	};
	uint8_t* m_pBuff[BUFF_TYPE_Num];

	//デバッグ
public:
	bool IsUseHeap() const
	{
		return m_dbgIsUseHeap;
	}

	size_t GetFreeByte()
	{
		AllBlockScopedLock lock(*this);

		size_t freeByte = 0;
		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_Pool_00];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		while (pEnd > pCurrent)
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			//未使用メモリ
			if (HEADER_REF_NUM == pBlockHeader->refIdx)
			{
				freeByte += pBlockHeader->size;
			}

			pCurrent += sizeof(BlockHeader) + pBlockHeader->size;
		}

		return freeByte;
	}

	int GetFreeBlockNum()
	{
		AllBlockScopedLock lock(*this);

		int freeBlockNum = 0;
		bool isHead = true;
		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_Pool_00];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		while (pEnd > pCurrent)
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			//未使用メモリ
			if (HEADER_REF_NUM == pBlockHeader->refIdx && isHead)
			{
				++freeBlockNum;
				isHead = false;
			}
			else
			{
				isHead = true;
			}

			pCurrent += sizeof(BlockHeader) + pBlockHeader->size;
		}

		return freeBlockNum;
	}
	int GetUseBlockNum()
	{
		AllBlockScopedLock lock(*this);

		int useBlockNum = 0;
		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_Pool_00];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		while (pEnd > pCurrent)
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			//使用メモリ
			if (HEADER_REF_NUM != pBlockHeader->refIdx)
			{
				++useBlockNum;
			}

			pCurrent += sizeof(BlockHeader) + pBlockHeader->size;
		}

		return useBlockNum;
	}

private:
	bool m_dbgIsUseHeap;

};

constexpr int THREAD_NUM = 10;
void test(MemoryPool& pool, int& count, std::mutex& countMutex, bool& isEnd, bool isUseComp, int compNum)
{
	constexpr int INIT_NUM = 120 / THREAD_NUM;
	constexpr int MAX_NUM = 180 / THREAD_NUM;
	std::array<MemoryPool::Handle<Test01>, MAX_NUM> test01Vec;
	std::array<MemoryPool::Handle<Test02>, MAX_NUM> test02Vec;
	std::array<MemoryPool::Handle<Test03>, MAX_NUM> test03Vec;
	std::array<MemoryPool::Handle<Test04>, MAX_NUM> test04Vec;
	std::array<MemoryPool::Handle<Test05>, MAX_NUM> test05Vec;
	std::array<MemoryPool::Handle<Test11>, MAX_NUM> test11Vec;
	std::array<MemoryPool::Handle<Test12>, MAX_NUM> test12Vec;
	std::array<MemoryPool::Handle<Test13>, MAX_NUM> test13Vec;
	std::array<MemoryPool::Handle<Test14>, MAX_NUM> test14Vec;
	std::array<MemoryPool::Handle<Test15>, MAX_NUM> test15Vec;

	for (int i = 0; i < INIT_NUM; ++i) { test01Vec[i] = pool.CreateObject<Test01>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test02Vec[i] = pool.CreateObject<Test02>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test03Vec[i] = pool.CreateObject<Test03>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test04Vec[i] = pool.CreateObject<Test04>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test05Vec[i] = pool.CreateObject<Test05>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test11Vec[i] = pool.CreateObject<Test11>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test12Vec[i] = pool.CreateObject<Test12>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test13Vec[i] = pool.CreateObject<Test13>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test14Vec[i] = pool.CreateObject<Test14>(); }
	for (int i = 0; i < INIT_NUM; ++i) { test15Vec[i] = pool.CreateObject<Test15>(); }

	std::random_device rand;
	while (true)
	{
		switch (rand() % 10)
		{
		case 0:
		{
			auto it = std::find_if(test01Vec.begin(), test01Vec.end(), [](MemoryPool::Handle<Test01>& v) {return v.IsValid(); });
			if (test01Vec.end() != it) { it->Release(); }
		}
		break;
		case 1:
		{
			auto it = std::find_if(test02Vec.begin(), test02Vec.end(), [](MemoryPool::Handle<Test02>& v) {return v.IsValid(); });
			if (test02Vec.end() != it) { it->Release(); }
		}
		break;
		case 2:
		{
			auto it = std::find_if(test03Vec.begin(), test03Vec.end(), [](MemoryPool::Handle<Test03>& v) {return v.IsValid(); });
			if (test03Vec.end() != it) { it->Release(); }
		}
		break;
		case 3:
		{
			auto it = std::find_if(test04Vec.begin(), test04Vec.end(), [](MemoryPool::Handle<Test04>& v) {return v.IsValid(); });
			if (test04Vec.end() != it) { it->Release(); }
		}
		break;
		case 4:
		{
			auto it = std::find_if(test05Vec.begin(), test05Vec.end(), [](MemoryPool::Handle<Test05>& v) {return v.IsValid(); });
			if (test05Vec.end() != it) { it->Release(); }
		}
		break;
		case 5:
		{
			auto it = std::find_if(test11Vec.begin(), test11Vec.end(), [](MemoryPool::Handle<Test11>& v) {return v.IsValid(); });
			if (test11Vec.end() != it) { it->Release(); }
		}
		break;
		case 6:
		{
			auto it = std::find_if(test12Vec.begin(), test12Vec.end(), [](MemoryPool::Handle<Test12>& v) {return v.IsValid(); });
			if (test12Vec.end() != it) { it->Release(); }
		}
		break;
		case 7:
		{
			auto it = std::find_if(test13Vec.begin(), test13Vec.end(), [](MemoryPool::Handle<Test13>& v) {return v.IsValid(); });
			if (test13Vec.end() != it) { it->Release(); }
		}
		break;
		case 8:
		{
			auto it = std::find_if(test14Vec.begin(), test14Vec.end(), [](MemoryPool::Handle<Test14>& v) {return v.IsValid(); });
			if (test14Vec.end() != it) { it->Release(); }
		}
		break;
		case 9:
		{
			auto it = std::find_if(test15Vec.begin(), test15Vec.end(), [](MemoryPool::Handle<Test15>& v) {return v.IsValid(); });
			if (test15Vec.end() != it) { it->Release(); }
		}
		break;
		}
		switch (rand() % 10)
		{
		case 0:
		{
			auto it = std::find_if(test01Vec.begin(), test01Vec.end(), [](MemoryPool::Handle<Test01>& v) {return v.IsInvalid(); });
			if (test01Vec.end() != it) { *it = pool.CreateObject<Test01>(); }
		}
		break;
		case 1:
		{
			auto it = std::find_if(test02Vec.begin(), test02Vec.end(), [](MemoryPool::Handle<Test02>& v) {return v.IsInvalid(); });
			if (test02Vec.end() != it) { *it = pool.CreateObject<Test02>(); }
		}
		break;
		case 2:
		{
			auto it = std::find_if(test03Vec.begin(), test03Vec.end(), [](MemoryPool::Handle<Test03>& v) {return v.IsInvalid(); });
			if (test03Vec.end() != it) { *it = pool.CreateObject<Test03>(); }
		}
		break;
		case 3:
		{
			auto it = std::find_if(test04Vec.begin(), test04Vec.end(), [](MemoryPool::Handle<Test04>& v) {return v.IsInvalid(); });
			if (test04Vec.end() != it) { *it = pool.CreateObject<Test04>(); }
		}
		break;
		case 4:
		{
			auto it = std::find_if(test05Vec.begin(), test05Vec.end(), [](MemoryPool::Handle<Test05>& v) {return v.IsInvalid(); });
			if (test05Vec.end() != it) { *it = pool.CreateObject<Test05>(); }
		}
		break;
		case 5:
		{
			auto it = std::find_if(test11Vec.begin(), test11Vec.end(), [](MemoryPool::Handle<Test11>& v) {return v.IsInvalid(); });
			if (test11Vec.end() != it) { *it = pool.CreateObject<Test11>(); }
		}
		break;
		case 6:
		{
			auto it = std::find_if(test12Vec.begin(), test12Vec.end(), [](MemoryPool::Handle<Test12>& v) {return v.IsInvalid(); });
			if (test12Vec.end() != it) { *it = pool.CreateObject<Test12>(); }
		}
		break;
		case 7:
		{
			auto it = std::find_if(test13Vec.begin(), test13Vec.end(), [](MemoryPool::Handle<Test13>& v) {return v.IsInvalid(); });
			if (test13Vec.end() != it) { *it = pool.CreateObject<Test13>(); }
		}
		break;
		case 8:
		{
			auto it = std::find_if(test14Vec.begin(), test14Vec.end(), [](MemoryPool::Handle<Test14>& v) {return v.IsInvalid(); });
			if (test14Vec.end() != it) { *it = pool.CreateObject<Test14>(); }
		}
		break;
		case 9:
		{
			auto it = std::find_if(test15Vec.begin(), test15Vec.end(), [](MemoryPool::Handle<Test15>& v) {return v.IsInvalid(); });
			if (test15Vec.end() != it) { *it = pool.CreateObject<Test15>(); }
		}
		break;
		}

		{
			std::lock_guard<std::mutex> lock(countMutex);

			if (isUseComp)
			{
				pool.Compaction(compNum);
			}

			++count;
		}

		if (pool.IsUseHeap())
		{
			isEnd = true;
		}
		if (isEnd)
		{
			break;//ヒープ使った時点で終了
		}
	}
}

void out(MemoryPool& pool, int& count, bool& isEnd, clock_t start)
{
	while (true)
	{
		system("cls");

		clock_t end = clock();

		size_t freeByte = pool.GetFreeByte();
		int freeBlockNum = pool.GetFreeBlockNum();
		int useBlockNum = pool.GetUseBlockNum();

		std::cout << "count: " << count << std::endl;
		std::cout << "freeByte: " << freeByte << std::endl;
		std::cout << "freeKB: " << freeByte / 1024 << std::endl;
		std::cout << "freeMB: " << freeByte / 1024 / 1024 << std::endl;
		std::cout << "freeBlock: " << freeBlockNum << std::endl;
		std::cout << "useBlock: " << useBlockNum << std::endl;
		std::cout << "time_ms: " << (static_cast<double>(end - start) / static_cast<double>(count)) << std::endl;


		if (isEnd)
		{
			break;//ヒープ使った時点で終了
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

int main()
{
#ifndef ALLOC_FREE_CHECK
	MemoryPool pool;

	constexpr bool isUseComp = true;
	constexpr int compNum = 80;
	bool isEnd = false;
	int count = 0;
	std::mutex countMutex;

	clock_t start = clock();

	std::vector<std::thread> threadVec;
	for (int i = 0; i < THREAD_NUM; ++i) 
	{
		threadVec.emplace_back(test, std::ref(pool), std::ref(count), std::ref(countMutex), std::ref(isEnd), isUseComp, compNum);
	}
	threadVec.emplace_back(out, std::ref(pool), std::ref(count), std::ref(isEnd), start);

	for (int i = 0; i < THREAD_NUM + 1; ++i) { threadVec[i].join(); }

	clock_t end = clock();

	//最終結果
	system("cls");

	size_t freeByte = pool.GetFreeByte();
	int freeBlockNum = pool.GetFreeBlockNum();
	int useBlockNum = pool.GetUseBlockNum();

	std::cout << "count: " << count << std::endl;
	std::cout << "freeByte: " << freeByte << std::endl;
	std::cout << "freeKB: " << freeByte / 1024 << std::endl;
	std::cout << "freeMB: " << freeByte / 1024 / 1024 << std::endl;
	std::cout << "freeBlock: " << freeBlockNum << std::endl;
	std::cout << "useBlock: " << useBlockNum << std::endl;
	std::cout << "time_ms: " << (static_cast<double>(end - start) / static_cast<double>(count)) << std::endl;

	system("pause");
#else
	MemoryPool pool;
	MemoryPool::Handle<Test01> handle1 = pool.CreateObject<Test01>();
	MemoryPool::Handle<Test01> handle2 = std::move(handle1);
	handle1.Release();
	handle2.Release();

#endif
}