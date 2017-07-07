
// Create a POST string the adds a data value to a feed.
//
// buffer:     character array the POST message will be inserted into.
// buffer_len: length of buffer in bytes.
// username:   null terminated string of the feed username.
// feedname:   null terminated string of the feed name.
// aio:        null terminated string of the AIO key.
// value:      null terminated string of the value to POST.
//
// Returns the length of the POST message.
int create_post_to_feed (char* buffer,
                         int buffer_len,
                         char* username,
                         char* feedname,
                         char* aio,
                         char* value);
