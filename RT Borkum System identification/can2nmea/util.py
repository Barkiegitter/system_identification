def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

def pretty_hex(binary_data):
    s = ''
    # chunk_str = []
    # for c in chunks(binary_data, 2):
    #     chunk_str.append(''.join('{:02x}'.format(x) for x in c))
    # return ' '.join(chunk_str)

    # Python 3 only
    return ' '.join(list(chunks(binary_data.hex(), 2)))