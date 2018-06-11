using Gtk, Graphics
c = @GtkCanvas()
win = GtkWindow(c, "Canvas")
gigi = 1
@guarded draw(c) do widget
        println("ok!")
        ctx = getgc(c)
        h = height(c)
        w = width(c)
        # Paint red rectangle
        if gigi == 1
                rectangle(ctx, 0, 0, w, h/2)
                set_source_rgb(ctx, 1, 0, 0)
                fill(ctx)
        end
        # Paint blue rectangle
        rectangle(ctx, 0, 3h/4, w, h/4)
        set_source_rgb(ctx, 0, 0, 1)
        fill(ctx)
end
show(c)
sleep(5.0)
gigi = 0
c.draw()
