"use strict";
    const fs = require("fs");
    const sizeOf = require('image-size');
    const path = "uploaded";
    const output = "../../themes/next/source/photos/photoslist.json";
    var dimensions;
    fs.readdir(path, function (err, files) {
        if (err) {
            return;
        }
        let arr = [];
        (function iterator(index) {
            
            
            if (index == files.length) {
                console.log(output)
                // modified by chatgpt
                fs.writeFile(output, JSON.stringify(arr, null, "\t"), (err) => {
                    if (err) {
                      console.error('写入文件时发生错误：', err);
                    } else {
                      console.log('文件写入成功。');
                    }
                  });
                return;
            }
            fs.stat(path + "/" + files[index], function (err, stats) {
                if (err) {
                    return;
                }
                if (stats.isFile()) {
                    dimensions = sizeOf(path + "/" + files[index]);
                    console.log(dimensions.width, dimensions.height);
                    arr.push(dimensions.width + '.' + dimensions.height + ' ' + files[index]);
                }
                iterator(index + 1);
            })
        }(0));
    });