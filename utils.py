def get_key_dict(dict,el):
    for key in dict:
        if el in dict[key]:
            return key



def set_non_none_value(arr, idx1, idx2):
    if arr[idx1] is not None:
        arr[idx2] = arr[idx1]
    elif arr[idx2] is not None:
        arr[idx1] = arr[idx2]
    return arr