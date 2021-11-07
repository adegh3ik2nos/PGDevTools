// mem_test.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>
#include <vector>
#include <random>
#include <array>
#include <mutex>
#include <thread>
#include <chrono>


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
		uint32_t isUseable : 1;
		uint32_t size : 31;
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
		Handle(MemoryPool* pPool, pointer_t* pHeaderRef)
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
		bool IsValid() const { return nullptr != m_pPool && nullptr != m_pHeaderRef; }
		bool IsInvalid() const { return !IsValid(); }

		void Release()
		{
			if (nullptr != m_pHeaderRef && nullptr != m_pPool)
			{
				m_pPool->_ReleaseBlock<Type>(m_pHeaderRef);

				//アクセス出来ないようにnullにする
				m_pHeaderRef = nullptr;
				m_pPool = nullptr;
			}
		}

	private:
		MemoryPool* m_pPool;
		pointer_t* m_pHeaderRef;
	};
	

public:
	MemoryPool()
		: m_pBuff{}
		, m_buffMutex()

		, m_dbgIsUseHeap(false)
	{
		std::lock_guard<std::mutex> lock(m_buffMutex);

		m_pBuff[BUFF_TYPE_Pool_00] = reinterpret_cast<uint8_t*>(std::calloc(POOL_SIZE_00, sizeof(uint8_t)));
		m_pBuff[BUFF_TYPE_HeaderRef] = reinterpret_cast<uint8_t*>(std::calloc(HEADER_REF_SIZE, sizeof(uint8_t)));

		//ヘッダも埋め込んでおく
		BlockHeader* pHeader = reinterpret_cast<BlockHeader*>(m_pBuff[BUFF_TYPE_Pool_00]);
		pHeader->isUseable = 1;
		pHeader->size = POOL_SIZE_00 - sizeof(BlockHeader);

		//ヘッダへの参照を保持
		pointer_t* pHeaderRef = reinterpret_cast<pointer_t*>(m_pBuff[BUFF_TYPE_HeaderRef]);
		*pHeaderRef = reinterpret_cast<pointer_t>(pHeader);
	}
	~MemoryPool()
	{
		std::lock_guard<std::mutex> lock(m_buffMutex);

		std::free(m_pBuff[BUFF_TYPE_HeaderRef]);
		std::free(m_pBuff[BUFF_TYPE_Pool_00]);
	}

public:
	template<typename Type, class... Args>
	Handle<Type> CreateObject(Args... args)
	{
		std::lock_guard<std::mutex> lock(m_buffMutex);

		//生成
		pointer_t* pUseableHeaderRef = _SearchUsealeHeaderRef();
		if (nullptr != pUseableHeaderRef)
		{
			BlockHeader* pUseableHeader = _SearchUseableHeader(sizeof(Type));
			if (nullptr != pUseableHeader)//プール使用
			{
				//ヘッダ情報書き換え
				uint8_t* pU8BlockHeader = reinterpret_cast<uint8_t*>(pUseableHeader);
				pUseableHeader->isUseable = 0;

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
					pNextBlockHeader->isUseable = 1;
					pNextBlockHeader->size = pUseableHeader->size - (sizeof(Type) + sizeof(BlockHeader));

					pUseableHeader->size = sizeof(Type);
				}

				//オブジェクト生成
				Type* pObject = new(pU8BlockHeader + sizeof(BlockHeader)) Type(args...);

				//ヘッダ参照
				*pUseableHeaderRef = reinterpret_cast<pointer_t>(pUseableHeader);
			}
			else//ヒープ使用
			{
				//オブジェクト生成
				Type* pObject = new Type(args...);

				//ヘッダ参照
				*pUseableHeaderRef = reinterpret_cast<pointer_t>(pObject);


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
		std::lock_guard<std::mutex> lock(m_buffMutex);

		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_Pool_00];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		BlockHeader* pUseableHeader = nullptr;
		size_t useableSize = 0;

		int currentNum = 0;
		while (pEnd > pCurrent)
		{
			BlockHeader* pHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			if (0 != pHeader->isUseable)
			{
				if (nullptr == pUseableHeader)
				{
					pUseableHeader = pHeader;
					useableSize = pHeader->size;
				}
				else
				{
					useableSize += sizeof(BlockHeader) + pHeader->size;
				}
			}
			else if(nullptr != pUseableHeader)
			{
				//未使用メモリ使って複製
				pUseableHeader->isUseable = 0;
				pUseableHeader->size = pHeader->size;
				std::memmove(
					reinterpret_cast<uint8_t*>(pUseableHeader) + sizeof(BlockHeader),
					reinterpret_cast<uint8_t*>(pHeader) + sizeof(BlockHeader),
					pHeader->size);

				//コンパクションした次のメモリブロック
				BlockHeader* pNextBlockHeader =
					reinterpret_cast<BlockHeader*>(reinterpret_cast<uint8_t*>(pUseableHeader) + sizeof(BlockHeader) + pUseableHeader->size);
				pNextBlockHeader->isUseable = 1;
				pNextBlockHeader->size = useableSize;

				//ヘッダの参照を差し替え
				pointer_t* pHeaderRefEnd = reinterpret_cast<pointer_t*>(m_pBuff[BUFF_TYPE_HeaderRef] + HEADER_REF_SIZE);
				pointer_t* pHeaderRef =
					std::find(
						reinterpret_cast<pointer_t*>(m_pBuff[BUFF_TYPE_HeaderRef]), 
						pHeaderRefEnd,
						reinterpret_cast<pointer_t>(pHeader));
				if (pHeaderRefEnd != pHeaderRef)
				{
					*pHeaderRef = reinterpret_cast<pointer_t>(pUseableHeader);
				}
				else
				{
					_ASSERT_EXPR(false, "ヘッダ参照が消えている");
				}

				pHeader = pUseableHeader;
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

			pCurrent += sizeof(BlockHeader) + pHeader->size;
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
		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_Pool_00];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		BlockHeader* pUseableHeader = nullptr;
		size_t useableSize = 0;

		while (pEnd > pCurrent)
		{
			BlockHeader* pHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			if (0 != pHeader->isUseable)
			{
				if (0 == useableSize)//連続未使用ブロックの先頭
				{
					pUseableHeader = pHeader;
					useableSize = pHeader->size;
				}
				else//連続未使用ブロックの先頭以外
				{
					useableSize += sizeof(BlockHeader) + pHeader->size;
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

			pCurrent += sizeof(BlockHeader) + pHeader->size;
		}

		return nullptr;
	}

	pointer_t* _SearchUsealeHeaderRef()
	{
		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_HeaderRef];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_HeaderRef] + HEADER_REF_SIZE;
		while (pEnd > pCurrent)
		{
			pointer_t* pHederRef = reinterpret_cast<pointer_t*>(pCurrent);

			//未使用メモリならヘッダの参照を戻す
			if (0 == *pHederRef)
			{
				return pHederRef;
			}

			pCurrent += sizeof(pointer_t);
		}

		return nullptr;
	}

	template<typename Type>
	void _ReleaseBlock(pointer_t* pHeaderRef)
	{
		std::lock_guard<std::mutex> lock(m_buffMutex);

		//破棄
		uint8_t* pU8BlockHeader = reinterpret_cast<uint8_t*>(*pHeaderRef);
		if (_IsPoolRange(pU8BlockHeader))
		{
			BlockHeader* pBlockHeader = reinterpret_cast<BlockHeader*>(pU8BlockHeader);
			if (0 < pBlockHeader->size)
			{
				Type* pObject = reinterpret_cast<Type*>(pU8BlockHeader + sizeof(BlockHeader));
				pObject->~Type();

				//ヘッダ情報の書き換え
				pBlockHeader->isUseable = 1;
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
		*pHeaderRef = 0;
	}

private:
	static constexpr size_t POOL_SIZE_00 = 1024 * 300;//プール０の最大バイト数
	static constexpr size_t HEADER_REF_SIZE = sizeof(pointer_t) * 3000;//ヘッダ参照領域の最大バイト数

	enum BUFF_TYPE : int
	{
		BUFF_TYPE_Pool_00, 
		BUFF_TYPE_HeaderRef, 

		BUFF_TYPE_Num
	};
	uint8_t* m_pBuff[BUFF_TYPE_Num];
	mutable std::mutex m_buffMutex;

	//デバッグ
public:
	bool IsUseHeap() const
	{
		return m_dbgIsUseHeap;
	}

	size_t GetFreeByte() const
	{
		std::lock_guard<std::mutex> lock(m_buffMutex);

		size_t freeByte = 0;
		uint8_t* pCurrent = m_pBuff[BUFF_TYPE_Pool_00];
		uint8_t* pEnd = m_pBuff[BUFF_TYPE_Pool_00] + POOL_SIZE_00;
		while (pEnd > pCurrent)
		{
			BlockHeader* pHeader = reinterpret_cast<BlockHeader*>(pCurrent);

			//未使用メモリ
			if (0 != pHeader->isUseable)
			{
				freeByte += pHeader->size;
			}

			pCurrent += sizeof(BlockHeader) + pHeader->size;
		}

		return freeByte;
	}

private:
	bool m_dbgIsUseHeap;

};

constexpr int THREAD_NUM = 1;
void test(MemoryPool& pool, int& count, std::mutex& countMutex, bool& isEnd, bool isUseCompaction)
{
	constexpr int INIT_NUM = 120 / THREAD_NUM;
	constexpr int MAX_NUM = 150 / THREAD_NUM;
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
			++count;

			if (isUseCompaction)
			{
				if (count % 10 == 0)
				{
					pool.Compaction(3);
				}
			}
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

void out(MemoryPool& pool, int& count, bool& isEnd)
{
	while (true)
	{
		system("cls");

		std::cout << "count: " << count << std::endl;
		std::cout << "freeByte: " << pool.GetFreeByte() << std::endl;
		std::cout << "freeKB: " << pool.GetFreeByte() / 1024 << std::endl;
		std::cout << "freeMB: " << pool.GetFreeByte() / 1024 / 1024 << std::endl;


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

	bool isEnd = false;
	int count = 0;
	std::mutex countMutex;

	std::vector<std::thread> threadVec;
	for (int i = 0; i < THREAD_NUM; ++i) 
	{
		threadVec.emplace_back(test, std::ref(pool), std::ref(count), std::ref(countMutex), std::ref(isEnd), false);
	}
	threadVec.emplace_back(out, std::ref(pool), std::ref(count), std::ref(isEnd));

	for (int i = 0; i < THREAD_NUM + 1; ++i) { threadVec[i].join(); }

	//最終結果
	system("cls");

	std::cout << "count: " << count << std::endl;
	std::cout << "freeByte: " << pool.GetFreeByte() << std::endl;
	std::cout << "freeKB: " << pool.GetFreeByte() / 1024 << std::endl;
	std::cout << "freeMB: " << pool.GetFreeByte() / 1024 / 1024 << std::endl;

	system("pause");
#else
	MemoryPool pool;
	MemoryPool::Handle<Test01> handle1 = pool.CreateObject<Test01>();
	MemoryPool::Handle<Test01> handle2 = std::move(handle1);
	handle1.Release();
	handle2.Release();

#endif
}

// プログラムの実行: Ctrl + F5 または [デバッグ] > [デバッグなしで開始] メニュー
// プログラムのデバッグ: F5 または [デバッグ] > [デバッグの開始] メニュー

// 作業を開始するためのヒント: 
//	1. ソリューション エクスプローラー ウィンドウを使用してファイルを追加/管理します 
//   2. チーム エクスプローラー ウィンドウを使用してソース管理に接続します
//   3. 出力ウィンドウを使用して、ビルド出力とその他のメッセージを表示します
//   4. エラー一覧ウィンドウを使用してエラーを表示します
//   5. [プロジェクト] > [新しい項目の追加] と移動して新しいコード ファイルを作成するか、[プロジェクト] > [既存の項目の追加] と移動して既存のコード ファイルをプロジェクトに追加します
//   6. 後ほどこのプロジェクトを再び開く場合、[ファイル] > [開く] > [プロジェクト] と移動して .sln ファイルを選択します
