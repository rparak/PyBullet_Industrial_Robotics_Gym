a = {}

a['Test_1'] = 1
a['Test_2'] = 2
a['Test_3'] = 3

if 'Test_1' in a.keys():
    print('Yes')
    
del a['Test_1']
print(a)

