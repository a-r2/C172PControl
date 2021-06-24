def my_func():
    datadict = {'a':1}
    globals().update(datadict)
    for key, value in datadict.items():
        exec('key = value')
    print(locals())
    print(a)
