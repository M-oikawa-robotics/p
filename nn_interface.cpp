#include <iostream>
#include "nn_interface.h"


static PyObject *module;


int initialize_NN(void)
{
    PyObject *init_func;
    PyObject *return_value;

    // Pythonインタプリタを起動，初期化する
    Py_Initialize();

    // モジュール読込時の検索先にカレントディレクトリを追加する
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('.')");
    PyRun_SimpleString("sys.argv = ['kyo_simulate.exe']");

    // pythonファイルをモジュールとしてインポートする
    module = PyImport_ImportModule("nn_interface");

    // モジュールがインポートできなければエラー
    if (module == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", "nn_interface");

        return 1;
    }

    // Python側のNN初期化関数を読み込む
    const char *funcname = "init_nn";
    init_func = PyObject_GetAttrString(module, funcname);

    // 関数が読み込めなければエラー
    if (!(init_func && PyCallable_Check(init_func)))
    {
        if (PyErr_Occurred())
        {
            PyErr_Print();
        }
        fprintf(stderr, "Cannot find function \"%s\"\n", funcname);

        return 1;
    }

    return_value = PyObject_CallObject(init_func, NULL);

    // 戻り値が NULL ならエラー
    if (return_value == NULL)
    {
        Py_DECREF(init_func);
        Py_DECREF(module);
        PyErr_Print();
        fprintf(stderr, "Call failed\n");
        return 1;
    }

    Py_DECREF(return_value);

    return 0;
}


int finalize_NN(void)
{
    Py_DECREF(module);

    // Pythonインタプリタを終了する
    Py_Finalize();

    return 0;
}


int input_state(const double state[8])
{
    PyObject *func, *args;

    const char *funcname = "input_state";
    int i;

    // モジュールから関数への参照を読み込む
    func = PyObject_GetAttrString(module, funcname);

    // 関数が読み込めなければエラー
    if (!(func && PyCallable_Check(func)))
    {
        if (PyErr_Occurred())
        {
            PyErr_Print();
        }
        fprintf(stderr, "Cannot find function \"%s\"\n", funcname);

        return 1;
    }

    // 引数を設定する
    PyObject *pList, *pValue;
    pList = PyList_New(0);
    for (i=0; i<8; i++)
    {
        pValue = PyFloat_FromDouble(state[i]);
        if (!pValue)
        {
            Py_DECREF(pList);
            Py_DECREF(module);
            fprintf(stderr, "Cannot convert argument\n");
            return 1;
        }
        PyList_Append(pList, pValue);
    }
    args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pList);

    // Pythonの関数を呼び出す
    PyObject *return_value;
    return_value = PyObject_CallObject(func, args);

    // 戻り値が NULL ならエラー
    if (return_value == NULL)
    {
        Py_DECREF(func);
        Py_DECREF(module);
        PyErr_Print();
        fprintf(stderr,"Call failed\n");
        return 1;
    }

    Py_DECREF(return_value);
    return 0;
}


int receive_action(double action[2])
{
    PyObject *func;

    const char *funcname = "receive_action";
    int i;

    // モジュールから関数への参照を読み込む
    func = PyObject_GetAttrString(module, funcname);

    // 関数が読み込めなければエラー
    if (!(func && PyCallable_Check(func)))
    {
        if (PyErr_Occurred())
        {
            PyErr_Print();
        }
        fprintf(stderr, "Cannot find function \"%s\"\n", funcname);

        return 1;
    }

    // Pythonの関数を呼び出す
    PyObject *return_value;
    return_value = PyObject_CallObject(func, NULL);

    // 戻り値が NULL ならエラー
    if (return_value == NULL)
    {
        Py_DECREF(func);
        Py_DECREF(module);
        PyErr_Print();
        fprintf(stderr, "Call failed\n");
        return 1;
    }

    // 戻り値がリスト型でなければエラー
    if (!PyList_Check(return_value))
    {
        Py_DECREF(return_value);
        Py_DECREF(func);
        Py_DECREF(module);
        PyErr_Print();
        fprintf(stderr, "Returned value is not a PyList\n");
        return 1;
    }

    // 戻り値を kyo_vector3d に変換する
    for (i=0; i<2; i++)
    {
        action[i] = PyFloat_AsDouble(PyList_GetItem(return_value, i));
    }

    Py_DECREF(return_value);
    return 0;
}


int quit_generate(void)
{
    PyObject *func;

    const char *funcname = "quit_generate";
    int i;

    // モジュールから関数への参照を読み込む
    func = PyObject_GetAttrString(module, funcname);

    // 関数が読み込めなければエラー
    if (!(func && PyCallable_Check(func)))
    {
        if (PyErr_Occurred())
        {
            PyErr_Print();
        }
        fprintf(stderr, "Cannot find function \"%s\"\n", funcname);

        return 1;
    }

    // Pythonの関数を呼び出す
    PyObject *return_value;
    return_value = PyObject_CallObject(func, NULL);

    // 戻り値が NULL ならエラー
    if (return_value == NULL)
    {
        Py_DECREF(func);
        Py_DECREF(module);
        PyErr_Print();
        fprintf(stderr, "Call failed\n");
        return 1;
    }

    Py_DECREF(return_value);
    return 0;
}


int is_end_of_motion(int *eom)
{
    PyObject *func;

    const char *funcname = "is_end_of_motion";
    int i;

    // モジュールから関数への参照を読み込む
    func = PyObject_GetAttrString(module, funcname);

    // 関数が読み込めなければエラー
    if (!(func && PyCallable_Check(func)))
    {
        if (PyErr_Occurred())
        {
            PyErr_Print();
        }
        fprintf(stderr, "Cannot find function \"%s\"\n", funcname);

        return 1;
    }

    // Pythonの関数を呼び出す
    PyObject *return_value;
    return_value = PyObject_CallObject(func, NULL);

    // 戻り値が NULL ならエラー
    if (return_value == NULL)
    {
        Py_DECREF(func);
        Py_DECREF(module);
        PyErr_Print();
        fprintf(stderr, "Call failed\n");
        return 1;
    }

    // 戻り値がリスト型でなければエラー
    if (!PyLong_Check(return_value))
    {
        Py_DECREF(return_value);
        Py_DECREF(func);
        Py_DECREF(module);
        PyErr_Print();
        fprintf(stderr, "Returned value is not a PyList\n");
        return 1;
    }

    // 戻り値を double に変換する
    *eom = PyLong_AsLong(return_value);

    Py_DECREF(return_value);
    return 0;
}
