from sklearn.linear_model import LinearRegression

X = [[0], [1], [2]]
Y = [[2, 3, 4, 5], [4, 6, 8, 10], [6, 9, 12, 15]]

lr = LinearRegression()
lr.fit(X, Y) # 学習

for i in range(4):
    print('coef = ', lr.coef_[i]) # 傾き
    print('intercept = ', lr.intercept_[i]) # 切片